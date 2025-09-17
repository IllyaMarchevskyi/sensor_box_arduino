// #include <Ethernet.h>

// // === RS-485 UART і керування напрямком ===
// #define RS485_UART      Serial3
// #define RS485_DIR_PIN   5  // твій DE/RE pin

// static inline void rs485_tx_on()  { digitalWrite(RS485_DIR_PIN, HIGH); }
// static inline void rs485_tx_off() { digitalWrite(RS485_DIR_PIN, LOW);  }

uint16_t mb_crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (uint8_t i=0;i<8;i++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

bool isInternetAlive(const IPAddress& testIp, uint16_t port, uint16_t timeoutMs=600) {
  Ethernet.maintain();                 // оновити DHCP-лізинг, якщо потрібно
  EthernetClient c;
  uint32_t t0 = millis();
  bool ok = c.connect(testIp, port);
  uint32_t dt = millis() - t0;
  if (ok) {
    Serial.println("Іnternet Сonnection Successful!!");
    c.stop();
    }
  // Можеш додати лог:
  Serial.print("TCP check: "); Serial.print(ok); Serial.print(" in "); Serial.print(dt); Serial.println(" ms");
  return ok;
}

bool relayTimedPulse(uint8_t unitId, uint8_t channel, uint16_t ticks100ms) {
  uint8_t frame[8];
  frame[0] = unitId;
  frame[1] = 0x05;             // vendor-спец. “Timed pulse” на 0x02xx (див. файл)
  frame[2] = 0x02;             // addr hi = 0x02
  frame[3] = channel;          // addr lo = номер каналу
  frame[4] = (uint8_t)(ticks100ms & 0xFF);       // value lo  (на прикладі — 07 00)
  frame[5] = (uint8_t)((ticks100ms >> 8) & 0xFF);// value hi

  uint16_t crc = mb_crc16(frame, 6);
  frame[6] = crc & 0xFF; frame[7] = crc >> 8;

  pre_transmission_main();
  size_t w = Serial3.write(frame, sizeof(frame));
  Serial3.flush();
  post_transmission_main();

  return (w == sizeof(frame));
}

bool relayWriteCoil(uint8_t unitId, uint8_t channel, bool on) {
  uint8_t frame[8];
  frame[0] = unitId;
  frame[1] = 0x05;             // Write Single Coil
  frame[2] = 0x00;             // addr hi
  frame[3] = channel;          // addr lo
  frame[4] = on ? 0xFF : 0x00; // data hi
  frame[5] = 0x00;             // data lo

  uint16_t crc = mb_crc16(frame, 6);
  frame[6] = crc & 0xFF; frame[7] = crc >> 8;

  size_t w = Serial3.write(frame, sizeof(frame));
  Serial3.flush();
  post_transmission_main();

  return (w == sizeof(frame));
}

void ensureNetOrRebootPort0(const IPAddress& testIp, uint16_t port) {
  if (millis() - relay < 2000) return;
  relay += 2000;
  

  if (isInternetAlive(testIp, port)) return;

  const uint8_t UNIT_ID = 0x0B;      // твій модуль реле з адресою 11
  const uint8_t CH      = 0;       // порт 0

  // ВАРІАНТ A (рекомендовано): короткий флеш-імпульс, напр. 5 * 100мс = 500мс
  if (!relayTimedPulse(UNIT_ID, CH, 5)) {
    // ВАРІАНТ B (fallback): OFF → 300мс → ON
    relayWriteCoil(UNIT_ID, CH, false);
    delay(300);
    relayWriteCoil(UNIT_ID, CH, true);
  }
}
