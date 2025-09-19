#include <ModbusMaster.h>
#include <Ethernet.h>

// Access to meteo data (if needed by mappings)
extern double meteo_dec[9];

// RS-485 arbitration helpers (defined in modules/utils.cpp)
extern bool rs485_acquire(uint16_t timeout_ms);
extern void rs485_release();

// Utils from modules/utils.cpp
size_t buildMbTcpRead03(uint8_t* out, uint16_t txId,
                        uint8_t unit, uint16_t addr, uint16_t qty);
void printHex(const uint8_t* b, size_t n);

// Global Modbus RTU master on Serial3
ModbusMaster sensor_box;
static uint32_t sensorbox_last_poll = 0;

static const char* hwName(uint8_t hs) {
  switch (hs) {
    case EthernetNoHardware: return "NoHardware";
    case EthernetW5100:      return "W5100";
    case EthernetW5200:      return "W5200";
    case EthernetW5500:      return "W5500";
    default:                 return "UnknownHW";
  }
}
static const char* linkName(uint8_t ls) {
  switch (ls) {
    case LinkON:   return "LinkON";
    case LinkOFF:  return "LinkOFF";
    default:       return "LinkUnknown";
  }
}
static const char* sockName(uint8_t s) {
  switch (s) {
    case 0x00: return "CLOSED";
    case 0x13: return "INIT";
    case 0x14: return "LISTEN";
    case 0x15: return "SYNSENT";
    case 0x16: return "SYNRECV";
    case 0x17: return "ESTABLISHED";
    case 0x18: return "FIN_WAIT";
    case 0x1A: return "CLOSING";
    case 0x1B: return "TIME_WAIT";
    case 0x1C: return "CLOSE_WAIT";
    case 0x1D: return "LAST_ACK";
    default:   return "OTHER";
  }
}

void printEthDiag(EthernetClient &c) {
  #if defined(ETHERNET_H)
    Serial.print(F("HW="));   Serial.print(hwName(Ethernet.hardwareStatus()));
    Serial.print(F(" LINK="));Serial.print(linkName(Ethernet.linkStatus()));
  #endif
  Serial.print(F(" Sock=0x")); Serial.print(c.status(), HEX);
  Serial.print(F(" (")); Serial.print(sockName(c.status())); Serial.println(F(")"));

  Serial.print(F("Local=")); Serial.print(Ethernet.localIP());
  Serial.print(F(" GW="));   Serial.print(Ethernet.gatewayIP());
  Serial.print(F(" DNS="));  Serial.print(Ethernet.dnsServerIP());
  Serial.print(F(" MASK=")); Serial.println(Ethernet.subnetMask());
}

bool pingId_Ethernet(const IPAddress& ip, uint16_t port, uint16_t timeoutMs = 800) {
  EthernetClient client;
  client.setTimeout(TCP_CONNECT_TIMEOUT_MS);
  Serial.print(F("Connect ")); Serial.print(ip); Serial.print(F(":")); Serial.println(port);
  #if defined(ETHERNET_H)
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println(F("ERROR: No Ethernet hardware"));
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println(F("ERROR: LinkOFF"));
    }
  #endif
  uint32_t t0 = millis();
  bool ok = client.connect(ip, port);
  uint32_t dt = millis() - t0;
  if (!ok) {
    Serial.print(F("Connect FAILED in ")); Serial.print(dt); Serial.println(F(" ms"));
    printEthDiag(client);
    client.stop();
    return false;
  }
  Serial.print(F("Connect OK in ")); Serial.print(dt); Serial.println(F(" ms"));
  printEthDiag(client);
  client.stop();
  return true;
}

static inline float floatFromWords(uint16_t high_word, uint16_t low_word) {
  uint32_t raw = ((uint32_t)low_word << 16) | high_word; // device-specific word order
  float f; memcpy(&f, &raw, sizeof(float));
  return f;
}

bool pingId(uint8_t id) {
  if (id == 4) return true; // handled via Ethernet ping externally
  if (!rs485_acquire(500)) return false;
  sensor_box.begin(id, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(20, 2);
  bool ok = (res == sensor_box.ku8MBSuccess);
  rs485_release();
  return ok;
}

bool read3Floats(uint8_t id, float out[3], uint16_t startAddr, uint16_t regCount) {
  if (!rs485_acquire(500)) return false;
  sensor_box.begin(id, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(startAddr, regCount);
  if (res != sensor_box.ku8MBSuccess) { rs485_release(); return false; }
  for (uint8_t i=0; i<3; ++i) {
    uint16_t hi = sensor_box.getResponseBuffer(i*2 + 0);
    uint16_t lo = sensor_box.getResponseBuffer(i*2 + 1);
    out[i] = floatFromWords(hi, lo);
  }
  rs485_release();
  return true;
}

bool read2Floats(uint8_t id, uint16_t startAddr, float &f0, float &f1) {
  if (!rs485_acquire(500)) return false;
  sensor_box.begin(id, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(startAddr, 4);
  if (res != sensor_box.ku8MBSuccess) { rs485_release(); return false; }
  uint16_t hi0 = sensor_box.getResponseBuffer(0);
  uint16_t lo0 = sensor_box.getResponseBuffer(1);
  uint16_t hi1 = sensor_box.getResponseBuffer(2);
  uint16_t lo1 = sensor_box.getResponseBuffer(3);
  f0 = floatFromWords(hi0, lo0);
  f1 = floatFromWords(hi1, lo1);
  rs485_release();
  return true;
}

void readTH_ID10(float* mass) {
  if (!rs485_acquire(500)) return;
  sensor_box.begin(11, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(0x0000, 2);
  if (res != sensor_box.ku8MBSuccess) {
    Serial.println("FAILED GET SERVICE_T DATA");
    rs485_release();
    return;
  }
  uint16_t rh_raw  = sensor_box.getResponseBuffer(0);
  uint16_t t_raw_u = sensor_box.getResponseBuffer(1);
  int16_t  t_raw_s = (int16_t)t_raw_u;
  mass[0] = t_raw_s / 10.0f;
  mass[1] = rh_raw / 10.0f;
  rs485_release();
}

float decodeFloat32(const uint8_t* p, bool wordSwap=true, bool byteSwap=false) {
  uint8_t b[4] = { p[0], p[1], p[2], p[3] };
  if (wordSwap) { uint8_t t0=b[0], t1=b[1]; b[0]=b[2]; b[1]=b[3]; b[2]=t0; b[3]=t1; }
  if (byteSwap) { uint8_t t=b[0]; b[0]=b[1]; b[1]=t; t=b[2]; b[2]=b[3]; b[3]=t; }
  uint32_t u = ((uint32_t)b[0]<<24)|((uint32_t)b[1]<<16)|((uint32_t)b[2]<<8)|b[3];
  float f; memcpy(&f, &u, 4);
  return f;
}

void sendHexTCP(float* mass, const IPAddress& ip, uint16_t port,
                const uint8_t* data, size_t len, uint16_t timeoutMs)
{
  EthernetClient client;
  client.setTimeout(TCP_CONNECT_TIMEOUT_MS);
  if (!pingId_Ethernet(ip, port)) return;
  client.connect(ip, port);

  size_t sent = client.write(data, len);
  client.flush();
  Serial.print(F("Sent ")); Serial.print(sent); Serial.println(F(" bytes:"));
  printHex(data, len);

  size_t got = 0;
  uint32_t t0 = millis();
  extern uint8_t resp[256];
  while (millis() - t0 < timeoutMs) {
    while (client.available() && got < 256) {
      resp[got++] = client.read();
    }
    if (!client.connected() && client.available() == 0) break;
  }
  client.stop();
  const uint8_t* dataStart = resp + 9;
  for (int i = 0; i < resp[8]/4; i++ )
    mass[i] = decodeFloat32(dataStart + i*4,  true, false);

  Serial.print(F("Recv ")); Serial.print(got); Serial.println(F(" bytes:"));
  if (got) printHex(resp, got);
  Serial.println(F("----------------------"));
}

void pollAllSensorBoxes(bool& alive2, bool& alive4, bool& alive6, bool& alive7) {
  alive2=alive4=alive6=alive7=false;
  // 1) Ping primary IDs
  for (uint8_t i=0; i<PRIMARY_COUNT; ++i) {
    uint8_t id = PRIMARY_IDS[i];
    bool ok = (id == 4) ? pingId_Ethernet(ip_4, port, time_sleep) : pingId(id);
    if      (id==2) alive2 = ok;
    else if (id==4) alive4 = ok;
    else if (id==6) alive6 = ok;
    else if (id==7) alive7 = ok;
  }

  // 2) Build list of additional IDs to poll
  uint8_t toPoll[7] = {0}; uint8_t nPoll = 0;
  if (alive2) for (uint8_t i=0;i<EXTRA_ONLY2_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY2[i];
  if (alive4) for (uint8_t i=0;i<EXTRA_ONLY4_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY4[i];
  if (alive6) for (uint8_t i=0;i<EXTRA_ONLY6_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY6[i];
  if (alive7) for (uint8_t i=0;i<EXTRA_ONLY7_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY7[i];

  // 3) Poll and map results into sensors_dec
  for (uint8_t i=0; i<nPoll; ++i) {
    uint8_t id = toPoll[i];
    float v[4] = {0};

    if (id == 6) {
      if (!read3Floats(id, v, GAS_START_ADDR, GAS_REG_COUNT)) continue;
      sensors_dec[5] = v[0]; // O3
      sensors_dec[3] = v[1]; // NO
      sensors_dec[4] = v[2]; // H2S
    } else if (id == 7) {
      if (!read3Floats(id, v, GAS_START_ADDR, GAS_REG_COUNT)) continue;
      sensors_dec[5] = v[0]; // O3
      sensors_dec[6] = v[1]; // NH3
      sensors_dec[4] = v[2]; // H2S
    } else if (id == 5) {
      if (!read3Floats(id, v, GAS_START_ADDR, GAS_REG_COUNT)) continue;
      sensors_dec[0] = v[0]; // CO
      sensors_dec[2] = v[1]; // NO2
      sensors_dec[1] = v[2]; // SO2
    } else if (id == 1) {
      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0032, /*qty*/4);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      sensors_dec[1] = v[0]; // SO2
      sensors_dec[4] = v[1]; // H2S
    } else if (id == 2) {
      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0031, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      sensors_dec[0] = v[0]; // CO
    } else if (id == 8) {
      float NO = 0, NO2 = 0;
      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0031, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      NO = v[0];
      len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0037, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      NO2 = v[0];
      len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x00b7, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      sensors_dec[3] = NO;   // NO
      sensors_dec[2] = NO2;  // NO2
      sensors_dec[6] = v[0]; // NH3
    }
  }

  // 4) PM via ID10
  float pm25=0, pm10=0;
  if (read2Floats(PM_ID, PM_START_ADDR, pm25, pm10)) {
    sensors_dec[7] = pm25;
    sensors_dec[8] = pm10;
  }
}
