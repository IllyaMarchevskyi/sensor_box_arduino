#define ARRLEN(a) (sizeof(a)/sizeof((a)[0]))
#define ENABLE_TIMING 1

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

// ---------- Pins ----------
constexpr uint8_t RS485_DIR_PIN = 5; // DE/RE for RS-485 (Sensor Box)
constexpr uint8_t BDBG_DIR_PIN = 4; // TX enable for BDBG line (if used)
constexpr uint8_t ENC28J60_CS = 10; // Ethernet CS


// ---------- UART speeds & formats ----------
constexpr uint32_t SERIAL0_BAUD = 115200; // USB debug
constexpr uint32_t SERIAL1_BAUD = 9600; // Meteo, 8N1
constexpr uint32_t SERIAL2_BAUD = 19200; // BDBG-09, 8N1
constexpr uint32_t SERIAL3_BAUD = 9600; // Sensor Box, 8E1 (adjust if your device differs)


// ---------- Sensor Box (Modbus RTU) ----------
const uint8_t PRIMARY_IDS[] = {2, 4, 6, 7};
const uint8_t PRIMARY_COUNT = ARRLEN(PRIMARY_IDS);

const uint8_t EXTRA_IF_ONLY2[]  = {5};
const uint8_t EXTRA_IF_ONLY4[]  = {1};
const uint8_t EXTRA_IF_ONLY6[]  = {6, 5};
const uint8_t EXTRA_IF_ONLY7[]  = {7, 5};
const uint8_t EXTRA_ONLY2_CNT   = ARRLEN(EXTRA_IF_ONLY2);
const uint8_t EXTRA_ONLY4_CNT   = ARRLEN(EXTRA_IF_ONLY4);
const uint8_t EXTRA_ONLY6_CNT   = ARRLEN(EXTRA_IF_ONLY6);
const uint8_t EXTRA_ONLY7_CNT   = ARRLEN(EXTRA_IF_ONLY7);

const uint8_t  PM_ID            = 10;
const uint8_t PM_START_ADDR     = 1; 
const uint8_t PM_REG_COUNT      = 4; 

// ID 4
const IPAddress ip_4(#, #, #, #);
const int port = 5581;
uint8_t time_sleep = 100;

constexpr uint16_t GAS_START_ADDR = 20; 
constexpr uint16_t GAS_REG_COUNT = 6; 

// ---------- Aggregation sizes ----------
constexpr uint8_t SAMPLES_PER_MIN = 60;   // accumulate 60 sec for minute avg
constexpr size_t  CH_COUNT        = 21;   // number of channels to aggregate




// ---------- Periods / Timings (ms) ----------
constexpr uint32_t TFT_REFRESH_MS = 60 * 1000;
constexpr uint32_t METEO_FRAME_SETTLE_MS = 500;
constexpr uint32_t BDBG_REQ_PERIOD_MS = 30 * 1000;
// constexpr uint32_t SENSORBOX_POLL_MS = 1000;
// constexpr uint32_t SENDARR_UPDATE_MS = 2000;


// ---------- Ethernet / Modbus TCP ----------
constexpr uint16_t MODBUS_TCP_PORT = 502;
static const byte MAC_ADDR[] = {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};
static const IPAddress STATIC_IP(192,168,0,101);
static const IPAddress DNS_IP(192,168,0,1);
uint8_t resp[256];



// ---------- Exported data array ----------
constexpr int SEND_ARR_SIZE = 30; // number of float32 values exposed (2 regs each)
constexpr float DEFAULT_SEND_VAL= -1.0f;


// Optional initial demo values (first 14 channels)
const char* const labels[] = {
  "CO", "SO2", "NO2", "NO", 
  "H2S", "O3", "NH3", "PM2.5", 
  "PM10", "WD", "TEMP", "RH", 
  "WS", "GST", "RAIN", "UV", 
  "LUX", "PRES", "R", "S_T", 
  "S_RH", "S_CO", "S_SO2", "S_NO2", 
  "S_NO", "S_H2S", "S_O3", "S_NH3", 
  };
constexpr size_t labels_len = sizeof(labels)/sizeof(labels[0]);

static const float INIT_SEND_ARR_0_13[][6] = {
  { -0.7,  -0.69, -0.71, -0.68, -0.72, -0.67 },
  {  0.1,   0.11,  0.09,  0.12,  0.08,  0.13 },
  {  0.01,  0.02,  0,     0.03, -0.01,  0.04 },
  {  0.2,   0.21,  0.19,  0.22,  0.18,  0.23 },
  { 22.6,  23.05, 22.15, 22.83, 22.37, 22.8  },
  { 56,    57.12, 54.88, 56.56, 55.44, 56.2  },
  {  4.76,  5,     4.52,  4.88,  4.64,  4.79 },
  {  5.6,   5.88,  5.32,  5.74,  5.46,  5.63 },
  { 28,    28.56, 27.44, 28.28, 27.72, 28.2  },
  {  3,     3.15,  2.85,  3.07,  2.92,  3.03 },
  { 540,  545.4, 534.6, 542.7, 537.3, 541    },
  {1010.22,1020.32,1000.12,1015.27,1005.17,1011.22},
  {  9,     9.45,  8.55,  9.22,  8.78,  9.03 },
  {  1.1,   1.16,  1.04,  1.13,  1.07,  1.13 },
  {  0.5,   0.51,  0.49,  0.52,  0.48,  0.53 },
  {  1,     1.05,  0.95,  1.02,  0.97,  1.03 },
  {  0,     0.01, -0.01,  0.02, -0.02,  0.03 },
  { 18,    18.36, 17.64, 18.18, 17.82, 18.2  },
  { 19,    19.38, 18.62, 19.19, 18.81, 19.2  },
  { 24.5,  24.99, 24.01, 24.75, 24.25, 24.7  },
  { 24.5,  24.99, 24.01, 24.75, 24.25, 24.7  }
};

