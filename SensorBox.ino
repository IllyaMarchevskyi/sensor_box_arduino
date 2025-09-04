bool pingId_Ethernet(const IPAddress& ip, uint16_t port,
                     uint16_t timeoutMs = 800) {
  
  EthernetClient client;
  Serial.print(F("Connect ")); Serial.print(ip); Serial.print(F(":")); Serial.println(port);

#if defined(ETHERNET_H)
  // швидка перевірка лінку/заліза
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println(F("ERROR: No Ethernet hardware (чи не ENC28J60 з невідповідною бібліотекою?)"));
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println(F("ERROR: LinkOFF (кабель/світч/живлення?)"));
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

void pollAllSensorBoxes(bool& alive2, bool& alive4, bool& alive6, bool& alive7) {
  // 1) Пінгуємо 2,3,6,7
  // bool alive2=false, alive4=false, alive6=false, alive7=false;
  Serial.println("Ping id");
  for (uint8_t i=0; i<PRIMARY_COUNT; ++i) {
    uint8_t id = PRIMARY_IDS[i];
    bool ok = false;
    Serial.print("ID "); Serial.println(id);
    if (id == 4) ok = pingId_Ethernet(ip_4, port, time_sleep);
    else ok = pingId(id);
    Serial.print("Status "); Serial.println(ok);
    
    if      (id==2) alive2 = ok;
    else if (id==4) alive4 = ok;
    else if (id==6) alive6 = ok;
    else if (id==7) alive7 = ok;
  }


  // 2) Формуємо список кого читати повністю
  uint8_t toPoll[7] = {0}; uint8_t nPoll = 0;
  if (alive2) for (uint8_t i=0;i<EXTRA_ONLY2_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY2[i];
  if (alive4) for (uint8_t i=0;i<EXTRA_ONLY4_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY4[i];
  if (alive6) for (uint8_t i=0;i<EXTRA_ONLY6_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY6[i];
  if (alive7) for (uint8_t i=0;i<EXTRA_ONLY7_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY7[i];

  // if (!alive2 && !alive34 && !alive6 && alive7) {
  //   for (uint8_t i=0;i<EXTRA_ONLY7_CNT;++i) toPoll[nPoll++] = EXTRA_IF_ONLY7[i];
  // }

  // 3) Читаємо гази й розкладаємо у sensors_dec
  for (uint8_t i=0; i<nPoll; ++i) {
    uint8_t id = toPoll[i];
    float v[4] = {0};


    if (!alive4 && !read3Floats(id, v, GAS_START_ADDR, GAS_REG_COUNT)) {
      Serial.print(F("ID ")); Serial.print(id); Serial.println(F(" read FAIL"));
      continue;
    } 

    // Мапінг під твої прилади:
    if (id == 6) {              // GA-100 (O3, NO, H2S)
      Serial.print("ID "); Serial.println(id);
      Serial.print("O3 "); Serial.println(v[0]);
      Serial.print("NO "); Serial.println(v[1]);
      Serial.print("H2S "); Serial.println(v[2]);
      sensors_dec[5] = v[0];    // O3
      sensors_dec[3] = v[1];    // NO
      sensors_dec[4] = v[2];    // H2S
    } else if (id == 7) {       // GA-100 (O3, NH3, H2S)
      Serial.print("ID "); Serial.println(id);
      Serial.print("O3 "); Serial.println(v[0]);
      Serial.print("NH3 "); Serial.println(v[1]);
      Serial.print("H2S "); Serial.println(v[2]);
      sensors_dec[5] = v[0];    // O3
      sensors_dec[6] = v[1];    // NH3
      sensors_dec[4] = v[2];    // H2S
    } else if (id == 5) {       // GA-100 (CO, NO2, SO2)
      Serial.print("ID "); Serial.println(id);
      Serial.print("CO "); Serial.println(v[0]);
      Serial.print("NO2 "); Serial.println(v[1]);
      Serial.print("SO2 "); Serial.println(v[2]);
      sensors_dec[0] = v[0];    // CO
      sensors_dec[2] = v[1];    // NO2
      sensors_dec[1] = v[2];    // SO2
    } else if (id == 2) {
      // TODO: ID2 (\"Дозор-с\") — вкажи, які саме 3 величини віддає і в якому порядку.
      // Напр.: sensors_dec[...] = v[0..2];
      Serial.print("ID "); Serial.println(id);
      Serial.print("CO "); Serial.println(v[0]);
      Serial.print("NO2 "); Serial.println(v[1]);
      Serial.print("SO2 "); Serial.println(v[2]);
      Serial.print("RH "); Serial.println(v[2]);
      sensors_dec[0] = v[0];    // CO
      sensors_dec[2] = v[1];    // NO2
      sensors_dec[1] = v[2];    // SO2
      meteo_dec[2] = v[2];    // SO2
    } else if (id == 1) {
      Serial.print("ID "); Serial.println(id);
      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0032, /*qty*/4);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      Serial.print("SO2 "); Serial.println(v[0]);
      Serial.print("H2S "); Serial.println(v[1]);
      sensors_dec[1] = v[0];    // SO2
      sensors_dec[4] = v[1];    // H2S

    } else if (id == 2) {
      Serial.print("ID "); Serial.println(id);
      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0031, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      Serial.print("CO "); Serial.println(v[0]);
      sensors_dec[0] = v[0];    // CO

    } else if (id == 8) {
      Serial.print("ID "); Serial.println(id);
      float NO = 0;
      float NO2 = 0;

      uint8_t REQ[12] = {0};
      size_t len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0031, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      NO = v[0];

      len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x0037, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);
      NO2 = v[0];

      len = buildMbTcpRead03(REQ, 0, /*id*/id, /*addr*/0x00b7, /*qty*/2);
      sendHexTCP(v, ip_4, port, REQ, len, time_sleep);

      Serial.print("NO "); Serial.println(v[0]);
      Serial.print("NO2 "); Serial.println(v[1]);
      Serial.print("NH3 "); Serial.println(v[2]);
      sensors_dec[3] = NO;    // NO
      sensors_dec[2] = NO2;    // NO2
      sensors_dec[6] = v[0];    // NH3

      
    }
  }

  // 4) PM з ID10 (якщо треба)
  float pm25=0, pm10=0;
  if (read2Floats(PM_ID, PM_START_ADDR, pm25, pm10)) {
    sensors_dec[7] = pm25;   // PM 2.5
    sensors_dec[8] = pm10;   // PM 10
  }
}

static inline float floatFromWords(uint16_t high_word, uint16_t low_word) {
  uint32_t raw = ((uint32_t)low_word << 16) | high_word; // word swap per device spec
  float f; memcpy(&f, &raw, sizeof(float));
  return f;
}

bool pingId(uint8_t id) {
  if (id == 4){
    
  } else {
    sensor_box.begin(id, Serial3);
    uint8_t res = sensor_box.readHoldingRegisters(GAS_START_ADDR, 2);
    return (res == sensor_box.ku8MBSuccess);
  }
}

bool read3Floats(uint8_t id, float out[3], uint16_t GAS_START_ADDR, uint16_t GAS_REG_COUNT) {
  sensor_box.begin(id, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(GAS_START_ADDR, GAS_REG_COUNT);
  if (res != sensor_box.ku8MBSuccess) return false;

  for (uint8_t i=0; i<3; ++i) {
    uint16_t hi = sensor_box.getResponseBuffer(i*2 + 0);
    uint16_t lo = sensor_box.getResponseBuffer(i*2 + 1);
    out[i] = floatFromWords(hi, lo);
  }
  return true;
}

bool read2Floats(uint8_t id, uint16_t startAddr, float &f0, float &f1) {
  sensor_box.begin(id, Serial3);
  uint8_t res = sensor_box.readHoldingRegisters(startAddr, 4);
  if (res != sensor_box.ku8MBSuccess) return false;
  uint16_t hi0 = sensor_box.getResponseBuffer(0);
  uint16_t lo0 = sensor_box.getResponseBuffer(1);
  uint16_t hi1 = sensor_box.getResponseBuffer(2);
  uint16_t lo1 = sensor_box.getResponseBuffer(3);
  f0 = floatFromWords(hi0, lo0);
  f1 = floatFromWords(hi1, lo1);
  return true;
}

// void sensorBoxPeriodicPoll() {
//   if (millis() - sensorbox_last_poll < SENSORBOX_POLL_MS) return;
//   sensorbox_last_poll = millis();

//   uint8_t res = sensor_box.readHoldingRegisters(SENSORBOX_START_ADDR, SENSORBOX_REG_COUNT);
//   if (res == sensor_box.ku8MBSuccess) {
//     for (uint8_t i = 0; i < SENSORBOX_REG_COUNT; i += 2) {
//       uint8_t outIdx = i / 2; // 0..2
//       uint16_t hi = sensor_box.getResponseBuffer(i);
//       uint16_t lo = sensor_box.getResponseBuffer(i + 1);
//       sensors_dec[outIdx] = floatFromWords(hi, lo);
//     }
//   } else {
//     Serial.print("SensorBox error: "); Serial.println(res, HEX);
//     post_transmission_main();
//   }
// }


void readTH_ID10(float* mass) {      // або інший baud, який ти записав у 0x07D1
  sensor_box.begin(11, Serial3);

  uint8_t res = sensor_box.readHoldingRegisters(0x0000, 2);  // FC 0x03
  if (res != sensor_box.ku8MBSuccess) {
    Serial.println("FAILED GET SERVERVISE_T DATA");
  };

  uint16_t rh_raw   = sensor_box.getResponseBuffer(0);  // %RH * 10
  uint16_t t_raw_u  = sensor_box.getResponseBuffer(1);  // °C * 10 (2’s complement)

  int16_t t_raw_s = (int16_t)t_raw_u;
  mass[0]  = t_raw_s / 10.0f;
  mass[1]     = rh_raw / 10.0f;

  Serial.print("T"); Serial.println(mass[0]);
  Serial.print("RH"); Serial.println(mass[1]);
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
  if (!pingId_Ethernet(ip, port)) return false;
  client.connect(ip, port);

  // Надсилаємо
  size_t sent = client.write(data, len);
  client.flush();
  Serial.print(F("Sent ")); Serial.print(sent); Serial.println(F(" bytes:"));
  printHex(data, len);

  // Читаємо відповідь до таймауту
  size_t got = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (client.available() && got < sizeof(resp)) {
      resp[got++] = client.read();
    }
    if (!client.connected() && client.available() == 0) break; // сервер закрив
  }
  client.stop();
  const uint8_t* dataStart = resp + 9;

  for (int i = 0; i < resp[8]/4; i++ )
    mass[i] = decodeFloat32(dataStart + i*4,  true, false); // ≈ 27.2531

  Serial.print(F("Recv ")); Serial.print(got); Serial.println(F(" bytes:"));
  if (got) printHex(resp, got);
  Serial.println(F("----------------------"));
  // Serial.print("V1: "); Serial.println(mass[0]);
  // Serial.print("V2: "); Serial.println(mass[1]);
  // Serial.print("V3: "); Serial.println(mass[2]);

}




