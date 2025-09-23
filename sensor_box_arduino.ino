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
// #include <ModbusMaster.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Ethernet.h>
#include <math.h>
#include "Config.h"

static double   send_arr[SEND_ARR_SIZE + 1] = {0};
static float    sensors_dec[9]              = {0};  // [CO, SO2, NO2, NO, H2S, O3, NH3, PM_2.5, PM_10]
double          meteo_dec[9]                = {0};  //
static float    radiation_uSvh              = 0.0f; //
static float    service_t[2]                = {0};  // temperature, dampness

static uint32_t main_timer           = 0;


#include "modules/utils.cpp"
#include "modules/relay.cpp"
#include "modules/sensor_box.cpp"
#include "modules/display.cpp"
#include "modules/ethernet.cpp"
#include "modules/ethernet_modbus.cpp"
#include "modules/meteo.cpp"
#include "modules/bdbg09.cpp"



// ================================== SETUP ==================================

void setup() {
  for (int i = 0; i < SEND_ARR_SIZE; i++) send_arr[i] = DEFAULT_SEND_VAL;

  initDisplay();
  Serial.begin(SERIAL0_BAUD);
  Serial.setTimeout(10);
  initSerials();

  pinMode(RS485_DIR_PIN, OUTPUT);
  pinMode(BDBG_DIR_PIN,  OUTPUT);

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
    
    // Sensor Box
    TIME_CALL("Sensor Box", pollAllSensorBoxes(alive2, alive4, alive6, alive7));

  }

  // BDBG-09
  if (!alive4){
  bdbgPeriodicRequest();
  while (Serial2.available()) { bdbgFeedByte(Serial2.read()); }
  bdbgTryFinalizeFrame();
  }

  // Meteo
  if (!alive4){
  while (Serial1.available()) { meteoFeedByte(Serial1.read()); }
  meteoTryFinalizeFrame();
  }

  TIME_CALL("Work with data", collectAndAverageEveryMinute());
  TIME_CALL("Modbus connect", modbusTcpServiceOnce());
  TIME_CALL("Drawing value on arduino", drawOnlyValue());
  TIME_CALL("Ralay", ensureNetOrRebootPort0());
  // TIME_CALL("Send to Server", httpPostSensors(SERVER_IP, 4000, "/ingest"));
  // TIME_CALL("Send to Server", httpPostHello(SERVER_IP, 4000, "/test"));
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



