/*
  Environmental Station — Refactored Main
  Modules:
    - TFT (TFT_eSPI / ST7796)
    - Meteo (Serial1 9600 8N1, 17-byte frame)
    - BDBG-09 (Serial2 19200 8N1, cmd 55 AA 01 → 10 bytes)
    - Sensor Box (RS-485 Modbus RTU on Serial3 8E1): CO/SO2/NO2 as float, start 20, len 6
    - Modbus TCP (ENC28J60) exposes send_arr[] as float32 (2 regs/value)

  This main keeps logic readable; all constants live in EnvStationConfig.h
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Ethernet.h>
#include <ModbusMaster.h>
#include <math.h>
#include "Config.h"

// ---------------- Display ----------------
TFT_eSPI tft;
static uint32_t tft_update_timer = 0;
static uint32_t sensor_box_timer = 0;
static uint32_t meteo_timer = 0;
static uint32_t bbdg_timer = 0;

// ---------------- Ethernet/Modbus TCP ----------------
EthernetServer server(MODBUS_TCP_PORT);

// send_arr: kept as double in RAM, sent as float32 on wire (explicit cast)
static double   send_arr[SEND_ARR_SIZE + 1] = {0};
static float    acc_sum[21] = {0};
static float    acc_sq_sum[21] = {0};
constexpr uint8_t SAMPLES_PER_MIN    = 60;
constexpr size_t CH_COUNT            = 21;
static uint16_t acc_count            = 0;
static uint32_t send_arr_last_update = 0;
static uint32_t last_sec_tick        = 0;

// ---------------- Meteo ----------------
static uint8_t  meteo_raw[21]       = {0};
static uint8_t  meteo_frame_buf[31] = {0};
static uint8_t  meteo_idx           = 0;
static uint32_t meteo_reset_ts      = 0;
static bool     meteo_has_frame     = false;
static double   meteo_dec[9]        = {0};

// ---------------- BDBG-09 ----------------
static float    radiation_uSvh = 0.0f;
static uint8_t  bdbg_raw[10]   = {0};
static uint8_t  bdbg_buf[20]   = {0};
static uint8_t  bdbg_idx       = 0;
static uint32_t bdbg_last_req  = 0;
static uint32_t bdbg_reset_ts  = 0;
static bool     bdbg_has_data  = false;

// ---------------- Sensor Box (Modbus RTU) ----------------
ModbusMaster sensor_box;
static uint32_t sensorbox_last_poll = 0;
static float    sensors_dec[9] = {0}; // [CO, SO2, NO2, NO, H2S, O3, NH3, PM_2.5, PM_10]
static float    service_t[2] = {0};

// ---------------- Prototypes ----------------
void initDisplay();
void initSerials();
void initEthernet();
void drawData();

// Meteo
void meteoFeedByte(uint8_t b);
void meteoTryFinalizeFrame();
void meteoDecodeToValues();

// BDBG-09
void bdbgPeriodicRequest();
void bdbgFeedByte(uint8_t b);
void bdbgTryFinalizeFrame();

// Sensor Box
void pollAllSensorBoxes();
static inline float floatFromWords(uint16_t hi, uint16_t lo);

// send_arr
void sendArrPeriodicUpdate();

// Modbus TCP
void modbusTcpServiceOnce();
void modbusTcpHandleRequest(EthernetClient &client, const uint8_t *req, size_t n);

// RS-485 dir
void pre_transmission_main();
void post_transmission_main();

// ================================== SETUP ==================================
int tmp_id_value = 0;
int main_timer = 0;
static uint32_t relay = 0;


void setup() {
  for (int i = 0; i < SEND_ARR_SIZE; i++) send_arr[i] = DEFAULT_SEND_VAL;

  initDisplay();
  Serial.begin(SERIAL0_BAUD);
  Serial.setTimeout(10);
  initSerials();

  pinMode(RS485_DIR_PIN, OUTPUT);
  pinMode(BDBG_DIR_PIN,  OUTPUT);

  sensor_box.begin(2, Serial3);
  sensor_box.preTransmission(pre_transmission_main);
  sensor_box.postTransmission(post_transmission_main);

  Serial.println("start - Arsenii'sTechnologies");
  initEthernet();
}

// ================================== LOOP ===================================
void loop() {
  bool alive2=false, alive4=false, alive6=false, alive7=false;
  uint32_t t1 = millis();

  if(millis() - main_timer >= 1000){
    main_timer = millis();


    // SERViSE Temperature
    TIME_CALL("Service t and rh", readTH_ID10(service_t));
    
    // // Sensor Box
    // Serial.println("Start Sensor Box");
    // TIME_CALL("SensorBox", pollAllSensorBoxes(alive2, alive4, alive6, alive7));
    // Serial.println("Finish Sensor Box");
    // // sendArrPeriodicUpdate();


  }

  // // BDBG-09
  // if (!alive4){
  // bdbgPeriodicRequest();
  // while (Serial2.available()) { bdbgFeedByte(Serial2.read()); }
  // bdbgTryFinalizeFrame();
  // }

  // // Meteo
  // if (!alive4){
  // while (Serial1.available()) { meteoFeedByte(Serial1.read()); }
  // meteoTryFinalizeFrame();
  // }

  IPAddress NET_CHECK_IP(8,8,8,8); // або твій сервер
  const uint16_t NET_CHECK_PORT = 53;
  TIME_CALL("Work with data", collectAndAverageEveryMinute());
  TIME_CALL("Modbus connect", modbusTcpServiceOnce());
  TIME_CALL("Drawing value on arduino", drawOnlyValue());
  TIME_CALL("Ralay", ensureNetOrRebootPort0(NET_CHECK_IP, NET_CHECK_PORT));
  uint32_t dt_ms = millis() - t1;
  if (dt_ms > 500) {Serial.print("Час: "); Serial.print(dt_ms); Serial.println(" ms");}

}

// =============================== Initialization ============================

void initSerials() {
  Serial1.begin(SERIAL1_BAUD);                // Meteo 8N1
  Serial1.setTimeout(10);

  Serial2.begin(SERIAL2_BAUD);                // BDBG 8N1
  Serial2.setTimeout(10);

  Serial3.begin(SERIAL3_BAUD);    // Sensor Box 8E1
  Serial3.setTimeout(10);
}



