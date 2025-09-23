static uint32_t relay = 0;
uint8_t req[] = {0x0B, 0x05, 0x02, 0x00, 0x64, 0x00};


uint16_t crc16_modbus(const uint8_t* p, size_t n);
void relayTimedPulse(uint8_t unitId, uint8_t channel, uint16_t ticks100ms);
bool isInternetAlive(const IPAddress& testIp, uint16_t port, uint16_t timeoutMs=600);

extern volatile bool g_rs485_busy;
bool rs485_acquire(uint16_t timeout_ms);
void rs485_release();

uint16_t crc16_modbus(const uint8_t* p, size_t n) {
  Serial.println("crc16_modbus");
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}


void relayTimedPulse(uint8_t unitId, uint8_t channel, uint16_t ticks100ms) {
  req[0] = (uint8_t)(unitId & 0xFF);
  req[3] = (uint8_t)(channel & 0xFF);
  req[4] = (uint8_t)(ticks100ms & 0xFF);

  uint16_t crc = crc16_modbus(req, sizeof(req));
  Serial.println("sendReq");
  if (!rs485_acquire(300)) {
    Serial.println("RS485 busy, skip relay pulse");
    return;
  }
  pre_transmission_main();
  Serial3.write(req, sizeof(req));
  Serial3.write(crc & 0xFF);
  Serial3.write((crc >> 8) & 0xFF);
  delay(20);
  post_transmission_main();
  rs485_release();
}

bool isInternetAlive(const IPAddress& testIp, uint16_t port, uint16_t timeoutMs=600) {
  Ethernet.maintain();                 
  EthernetClient c;
  uint32_t t0 = millis();
  bool ok = c.connect(testIp, port);
  uint32_t dt = millis() - t0;
  if (ok) {
    Serial.println("Іnternet Сonnection Successful!!");
    c.stop();
    }
  Serial.print("TCP check: "); Serial.print(ok); Serial.print(" in "); Serial.print(dt); Serial.println(" ms");
  return ok;
}

void ensureNetOrRebootPort0() {
  if (millis() - relay < RELAY_SLEEP) return;
  relay = millis();
  
  if (isInternetAlive(NET_CHECK_IP, NET_CHECK_PORT)) return;

  relayTimedPulse(UNIT_ID, CH, 5);
}

