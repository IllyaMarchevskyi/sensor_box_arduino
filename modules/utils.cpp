#include <Arduino.h>

constexpr size_t CH_COUNT     = 21;
int tmp_id_value              = 0;
static uint16_t acc_count            = 0;
static uint32_t last_sec_tick        = 0;

static float    acc_sum[21] = {0};
static float    acc_sq_sum[21] = {0};
constexpr uint8_t SAMPLES_PER_MIN    = 60;

volatile bool g_rs485_busy = false;


#define ENABLE_TIMING 1

#if ENABLE_TIMING
  #define TIME_CALL(label, call_expr) do {                        \
    uint32_t __t0  = micros();                                    \
    uint32_t __t0m = millis();                                    \
    (void)(call_expr);                                            \
    uint32_t __dus = micros() - __t0;                             \
    uint32_t __dms = millis() - __t0m;                            \
    if (__dms > 100) {                                            \
      Serial.print(F("[T] ")); Serial.print(label);               \
      Serial.print(F(": "));  Serial.print(__dus);                \
      Serial.print(F(" us  (~")); Serial.print(__dms);            \
      Serial.println(F(" ms)"));                                  \
    }                                                             \
  } while (0)
#else
  #define TIME_CALL(label, call_expr) (void)(call_expr)
#endif

bool rs485_acquire(uint16_t timeout_ms=300) {
  uint32_t t0 = millis();
  while (g_rs485_busy) {
    if (millis() - t0 > timeout_ms) return false;
    delay(1);
  }
  g_rs485_busy = true;
  return true;
}

void rs485_release() {
  g_rs485_busy = false;
  delayMicroseconds(4000);
}


size_t buildMbTcpRead03(uint8_t* out, uint16_t txId,
                        uint8_t unit, uint16_t addr, uint16_t qty);
void printHex(const uint8_t* b, size_t n);
void arrSumPeriodicUpdate();
void sendArrPeriodicUpdate();
void pre_transmission_main();
void post_transmission_main();
void collectAndAverageEveryMinute();



size_t buildMbTcpRead03(uint8_t* out, uint16_t txId,
                        uint8_t unit, uint16_t addr, uint16_t qty) {
  out[0]=txId>>8; out[1]=txId;     // TxID
  out[2]=0; out[3]=0;              // Protocol = 0
  out[4]=0; out[5]=6;              // Length = 6 (Unit+PDU)
  out[6]=unit;                     // Unit ID
  out[7]=0x03;                     // Function
  out[8]=addr>>8; out[9]=addr;     // Start address
  out[10]=qty>>8; out[11]=qty;     // Quantity
  return 12;
}

void printHex(const uint8_t* b, size_t n) {
  for (size_t i=0;i<n;i++){ if(b[i]<0x10) Serial.print('0'); Serial.print(b[i],HEX); Serial.print(' '); }
  Serial.println();
}


void arrSumPeriodicUpdate() {


  acc_sum[0]  += sensors_dec[0]; // CO
  acc_sq_sum[0] += sensors_dec[0] * sensors_dec[0];

  acc_sum[1]  += sensors_dec[1]; // SO2
  acc_sq_sum[1] += sensors_dec[1] * sensors_dec[1];

  acc_sum[2]  += sensors_dec[2]; // NO2
  acc_sq_sum[2] += sensors_dec[2] * sensors_dec[2];

  acc_sum[3]  += sensors_dec[3]; // NO
  acc_sq_sum[3] += sensors_dec[3] * sensors_dec[3];

  acc_sum[4]  += sensors_dec[4]; // H2S
  acc_sq_sum[4] += sensors_dec[4] * sensors_dec[4];

  acc_sum[5]  += sensors_dec[5]; // O3
  acc_sq_sum[5] += sensors_dec[5] * sensors_dec[5];

  acc_sum[6]  += sensors_dec[6]; // NH3
  acc_sq_sum[6] += sensors_dec[6] * sensors_dec[6];

  acc_sum[7]  += sensors_dec[7]; // PM_2.5
  acc_sq_sum[7] += sensors_dec[7] * sensors_dec[7];

  acc_sum[8]  += sensors_dec[8]; // PM_10
  acc_sq_sum[8] += sensors_dec[8] * sensors_dec[8];

  // Метео-дані
  acc_sum[9]  += meteo_dec[0]; // Wind direction
  acc_sq_sum[9] += meteo_dec[0] * meteo_dec[0];

  acc_sum[10] += meteo_dec[1]; // Temperature
  acc_sq_sum[10] += meteo_dec[1] * meteo_dec[1];

  acc_sum[11] += meteo_dec[2]; // Humidity (RH)
  acc_sq_sum[11] += meteo_dec[2] * meteo_dec[2];

  acc_sum[12] += meteo_dec[3]; // Wind speed
  acc_sq_sum[12] += meteo_dec[3] * meteo_dec[3];

  acc_sum[13] += meteo_dec[4]; // Gust
  acc_sq_sum[13] += meteo_dec[4] * meteo_dec[4];

  acc_sum[14] += meteo_dec[5]; // Rainfall
  acc_sq_sum[14] += meteo_dec[5] * meteo_dec[5];

  acc_sum[15] += meteo_dec[6]; // UV index
  acc_sq_sum[15] += meteo_dec[6] * meteo_dec[6];

  acc_sum[16] += meteo_dec[7]; // Light
  acc_sq_sum[16] += meteo_dec[7] * meteo_dec[7];

  acc_sum[17] += meteo_dec[8]; // Pressure
  acc_sq_sum[17] += meteo_dec[8] * meteo_dec[8];

  // Радіація
  acc_sum[18] += radiation_uSvh;
  acc_sq_sum[18] += radiation_uSvh * radiation_uSvh;

  // Температура всередині пристрою 
  acc_sum[19] += service_t[0];
  acc_sq_sum[19] += service_t[0] * service_t[0];

  acc_sum[20] += service_t[1];
  acc_sq_sum[20] += service_t[1] * service_t[1];

  //   for (size_t i = 0; i < CH_COUNT; ++i) {
  //   float value = INIT_SEND_ARR_0_13[i][tmp_id_value%6];
  //   acc_sum[i] += value;  // double -> float ок
  //   acc_sq_sum[i] += value * value;
  // }

  for(int id=0; id<18; id++)
  {
    Serial.print(labels[id]);
    Serial.print(": ");
    Serial.print("acc_sum");
    Serial.print('[');
    Serial.print(id);
    Serial.print("]=");
    Serial.print(acc_sum[id]);
    Serial.println(';');
  }
}

// ============================== send_arr Maintenance ========================
void sendArrPeriodicUpdate() {
  for(int id=0; id<labels_len; id++)
  {
    Serial.print(labels[id]);
    Serial.print(": ");
    Serial.print("send_arr");
    Serial.print('[');
    Serial.print(id);
    Serial.print("]=");
    Serial.print(send_arr[id]);
    Serial.println(';');
  }
}

// ================================ RS-485 DIR ================================
void pre_transmission_main()  { digitalWrite(RS485_DIR_PIN, HIGH); }
void post_transmission_main() { digitalWrite(RS485_DIR_PIN, LOW);  }

void collectAndAverageEveryMinute() {
  if (millis() - last_sec_tick < 1000) return;
  Serial.print("Last_sec_tick: "); Serial.println(last_sec_tick);
  last_sec_tick = millis();

  arrSumPeriodicUpdate();
  acc_count++;

  if (acc_count >= SAMPLES_PER_MIN) {
    for (size_t index = 0; index < CH_COUNT; ++index) {
      float tmp = acc_sum[index] / acc_count;
      send_arr[index] = tmp; // середнє за хвилину
      
      if (index < labels_len - CH_COUNT) {
        Serial.print("N: "); Serial.println(SAMPLES_PER_MIN);
        Serial.print("H: "); Serial.println(tmp*tmp);
        Serial.print("ix^2"); Serial.println(acc_sq_sum[index]);
        float tmp2 = (acc_sq_sum[index]-SAMPLES_PER_MIN*tmp*tmp)/(SAMPLES_PER_MIN-1);
        if (tmp2 < 0.0f) tmp2 = 0.0f;
        send_arr[CH_COUNT+index] = sqrtf(tmp2);
        }
      acc_sum[index]  = 0;            
      acc_sq_sum[index] = 0;           // готуємося до наступної хвилини
    }
    acc_count = 0;

    Serial.println(F("--- 1-min averages ready ---"));
    sendArrPeriodicUpdate();
  }
  tmp_id_value ++;

}
