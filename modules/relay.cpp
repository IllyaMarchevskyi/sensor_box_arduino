bool time_guard_allow(const char* key, uint32_t interval_ms);
void printHex(const uint8_t* b, size_t n);
void pre_transmission_main();
void post_transmission_main();

uint16_t crc16_modbus(const uint8_t* p, size_t n);
void relayTimedPulse(uint8_t unitId, uint8_t channel);
bool isInternetAlive(const IPAddress& testIp, uint16_t port, uint16_t timeoutMs=600);

extern volatile bool g_rs485_busy;
bool rs485_acquire(uint16_t timeout_ms);
void rs485_release();

uint16_t crc16_modbus(const uint8_t* p, size_t n) {
  // Modbus RTU CRC16
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

static inline void build_coil_cmd(uint8_t unit, uint16_t addr, bool on, uint8_t out[8]) {
  out[0] = unit;
  out[1] = 0x05;
  out[2] = (uint8_t)(addr >> 8);
  out[3] = (uint8_t)(addr & 0xFF);
  out[4] = on ? 0xFF : 0x00;
  out[5] = 0x00;
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = (uint8_t)(crc & 0xFF);
  out[7] = (uint8_t)((crc >> 8) & 0xFF);
}

// 0x06 Write Single Register (some relay boards use registers instead of coils)
static inline void build_reg06_cmd(uint8_t unit, uint16_t addr, uint16_t value, uint8_t out[8]) {
  out[0] = unit;
  out[1] = 0x06;
  out[2] = (uint8_t)(addr >> 8);
  out[3] = (uint8_t)(addr & 0xFF);
  out[4] = (uint8_t)(value >> 8);
  out[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = (uint8_t)(crc & 0xFF);
  out[7] = (uint8_t)((crc >> 8) & 0xFF);
}

static size_t rs485_send_recv(const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_max, uint16_t timeout_ms) {
  // TX phase
  pre_transmission_main();
  delayMicroseconds(1200);
  Serial3.write(tx, tx_len);
  Serial3.flush();
  delayMicroseconds(8000);
  post_transmission_main();

  // RX phase (best-effort)
  size_t got = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms && got < rx_max) {
    while (Serial3.available() && got < rx_max) {
      rx[got++] = Serial3.read();
    }
  }
  return got;
}


void relayTimedPulse(uint8_t unitId, uint8_t channel) {
  uint8_t frame[8];
  uint8_t resp[32];

  if (!rs485_acquire(500)) {
    Serial.println("RS485 busy, skip relay pulse");
    return;
  }

  // OFF
  Serial.println("Stop Relay");
  build_coil_cmd(unitId, (uint16_t)channel, /*on=*/false, frame);
  size_t r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("OFF tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("OFF rx: "); printHex(resp, r); }

  // Keep relay OFF for 2 seconds
  delay(1000);

  // ON via 0x05
  Serial.println("Start Relay");
  build_coil_cmd(unitId, (uint16_t)channel, /*on=*/true, frame);
  r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("ON 0x05 tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("ON 0x05 rx: "); printHex(resp, r); }

  // Try ON at addr 0 with standard 0x05 (some boards use 0 as global relay)
  delay(40);
  build_coil_cmd(unitId, 0x0000, /*on=*/true, frame);
  r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("ON 0x05 addr0 tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("ON 0x05 addr0 rx: "); printHex(resp, r); }

  // Some non-compliant boards expect 0x00FF for ON (byte-swapped)
  delay(40);
  frame[0] = unitId; frame[1]=0x05; frame[2]=0; frame[3]=(uint8_t)channel; frame[4]=0x00; frame[5]=0xFF;
  {
    uint16_t crc2 = crc16_modbus(frame, 6);
    frame[6] = (uint8_t)(crc2 & 0xFF);
    frame[7] = (uint8_t)((crc2 >> 8) & 0xFF);
  }
  r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("ON 0x05 swapped tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("ON 0x05 swapped rx: "); printHex(resp, r); }

  // Compatibility: some relays expect 0x06 register write
  delay(60);
  build_reg06_cmd(unitId, (uint16_t)channel, 0x0001, frame);
  r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("ON 0x06 ch val=1 tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("ON 0x06 ch rx: "); printHex(resp, r); }

  delay(40);
  build_reg06_cmd(unitId, 0x0000, 0x0001, frame);
  r = rs485_send_recv(frame, sizeof(frame), resp, sizeof(resp), 80);
  Serial.print("ON 0x06 addr0 val=1 tx: "); printHex(frame, sizeof(frame));
  if (r) { Serial.print("ON 0x06 addr0 rx: "); printHex(resp, r); }

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
  if (!time_guard_allow("relay", RELAY_SLEEP)) return;
  
  if (isInternetAlive(NET_CHECK_IP, NET_CHECK_PORT)) return;

  relayTimedPulse(UNIT_ID, CH);
}
