

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

// bool pingId_Ethernet(м& client, const IPAddress& ip, uint16_t port,
//                      uint16_t timeoutMs = 800) {
//   Serial.print(F("Connect ")); Serial.print(ip); Serial.print(F(":")); Serial.println(port);

// #if defined(ETHERNET_H)
//   // швидка перевірка лінку/заліза
//   if (Ethernet.hardwareStatus() == EthernetNoHardware) {
//     Serial.println(F("ERROR: No Ethernet hardware (чи не ENC28J60 з невідповідною бібліотекою?)"));
//   }
//   if (Ethernet.linkStatus() == LinkOFF) {
//     Serial.println(F("ERROR: LinkOFF (кабель/світч/живлення?)"));
//   }
// #endif

//   uint32_t t0 = millis();
//   bool ok = client.connect(ip, port);
//   uint32_t dt = millis() - t0;

//   if (!ok) {
//     Serial.print(F("Connect FAILED in ")); Serial.print(dt); Serial.println(F(" ms"));
//     printEthDiag(client);
//     client.stop();
//     return false;
//   }

//   Serial.print(F("Connect OK in ")); Serial.print(dt); Serial.println(F(" ms"));
//   printEthDiag(client);
//   return true;
// }

void arrSumPeriodicUpdate() {
  // if (millis() - send_arr_last_update < SENDARR_UPDATE_MS) return;
  // send_arr_last_update = millis();

  for (size_t i = 0; i < CH_COUNT; ++i) {
    float value = INIT_SEND_ARR_0_13[i][tmp_id_value%6];
    acc_sum[i] += value;  // double -> float ок
    acc_sq_sum[i] += value * value;
  }
  acc_count++;
  // Example mapping (uncomment to expose live daаta):
  // acc_sum[0]  = sensors_dec[0]; // CO
  // acc_sum[1]  = sensors_dec[1]; // SO2
  // acc_sum[2]  = sensors_dec[2]; // NO2
  // acc_sum[3]  = sensors_dec[3]; // NO
  // acc_sum[4]  = sensors_dec[4]; // H2S
  // acc_sum[5]  = sensors_dec[5]; // O3
  // acc_sum[6]  = sensors_dec[6]; // NH3
  // acc_sum[7]  = sensors_dec[7]; // PM_2.5
  // acc_sum[8]  = sensors_dec[8]; // PM_10
  // acc_sum[9]  = meteo_dec[0];   // wind dir
  // acc_sum[10]  = meteo_dec[1];   // temp
  // acc_sum[11]  = meteo_dec[2];   // RH
  // acc_sum[12]  = meteo_dec[3];   // wind
  // acc_sum[13]  = meteo_dec[4];   // gust
  // acc_sum[14]  = meteo_dec[5];   // rainfall
  // acc_sum[15]  = meteo_dec[6];   // UV
  // acc_sum[16] = meteo_dec[7];   // light
  // acc_sum[17] = meteo_dec[8];   // pressure
  // acc_sum[18] = radiation_uSvh; // radiation
  // acc_sum[19] = 22.5;           // internal temp (placeholder)
  // acc_sum[20] = 22.5;           // internal temp (placeholder)
  Serial.println();
  for(int id=0; id< 21; id++)
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
  Serial.println();
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
  // Раз на секунду
  if (millis() - last_sec_tick < 1000) return;
  Serial.print("Last_sec_tick: "); Serial.println(last_sec_tick);
  last_sec_tick += 1000; // так тримаємо стабільний ритм

  // 1) Накопичуємо суму по кожному з CH_COUNT каналів
  arrSumPeriodicUpdate();
  // for (size_t i = 0; i < CH_COUNT; ++i) {
  //   acc_sum[i] += (float)send_arr[i];  // double -> float ок
  // }
  // acc_count++;

  // 2) Коли набігло 60 зразків — рахуємо середнє, скидаємо суматор
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

    // (опційно) вивести результати
    Serial.println(F("--- 1-min averages ready ---"));
    sendArrPeriodicUpdate();
  }
  tmp_id_value ++;

}
