
//######### VERSION #########
#define VERSION "1.1.2"
//###########################

/*
  Environmental Station — Refactored Main
  Modules:
    - TFT (TFT_eSPI / ST7796)
    - BDBG-09 (Serial2 19200 8N1, cmd 55 AA 01 → 10 bytes)
    - Sensor Box (RS-485 Modbus RTU on Serial3 ): CO/SO2/NO2 as float, start 20, len 6
    - Modbus TCP (ENC28J60) exposes send_arr[] as float32 (2 regs/value)

  This main keeps logic readable; all constants live in EnvStationConfig.h
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include <Ethernet.h>
#include <math.h>
#include "Config.h"


static double   send_arr[SEND_ARR_SIZE + 1] = {0};
static float    sensors_dec[9]              = {0};  // [CO, SO2, NO2, NO, H2S, O3, NH3, PM_2.5, PM_10]
static float    radiation_uSvh              = 0.0f; //
static float    service_t[2]                = {0};  // temperature, dampness

static uint32_t main_timer           = 0;


#include "modules/utils.cpp"
#include "modules/relay.cpp"
#include "modules/sensor_box.cpp"
#include "modules/display.cpp"
#include "modules/ethernet.cpp"
#include "modules/ethernet_modbus.cpp"
#include "modules/bdbg09.cpp"



// ================================== SETUP ==================================

void setup() {
  for (int i = 0; i < SEND_ARR_SIZE; i++) send_arr[i] = DEFAULT_SEND_VAL;

  initDisplay();
  Serial.begin(SERIAL0_BAUD);
  Serial.setTimeout(10);
  Serial.println("Setup Monitoring");
  
  initSerials();

  pinMode(RS485_DIR_PIN, OUTPUT);
  pinMode(BDBG_DIR_PIN,  OUTPUT);
  digitalWrite(BDBG_DIR_PIN, LOW);

  sensor_box.preTransmission(pre_transmission_main);
  sensor_box.postTransmission(post_transmission_main);

  initEthernet();
  Serial.println("Finsh Initialization");
}
void loop() {
  bool alive2=false, alive4=false, alive6=false, alive7=false;
  uint32_t t1 = millis();

  if(millis() - main_timer >= 1000){
    main_timer = millis();
    // SERViSE Temperature
    TIME_CALL("Service t and rh", read_TEMP_RH(service_t));
    
    // Sensor Box
    TIME_CALL("Sensor Box", pollAllSensorBoxes(alive2, alive4, alive6, alive7));

  }

  // BDBG-09
  if (!alive4){
  bdbgPeriodicRequest();
  while (Serial2.available()) { bdbgFeedByte(Serial2.read()); }
  bdbgTryFinalizeFrame();
  }

  TIME_CALL("Work with data", collectAndAverageEveryMinute());
  TIME_CALL("Modbus connect", modbusTcpServiceOnce());
  TIME_CALL("Drawing value on arduino", drawOnlyValue());
  TIME_CALL("Ralay", ensureNetOrRebootPort0());
  TIME_CALL("Send to Server1", httpPostSensors(SERVER_IP, server_port, "/ingest"));
  uint32_t dt_ms = millis() - t1;
  if (dt_ms > 500) {Serial.print("Час: "); Serial.print(dt_ms); Serial.println(" ms");}

}

// =============================== Initialization ============================

void initSerials() {
  Serial2.begin(SERIAL2_BAUD);                // BDBG 8N1
  Serial2.setTimeout(10);

  Serial3.begin(SERIAL3_BAUD);    // Sensor Box 8E1
  Serial3.setTimeout(10);
}



