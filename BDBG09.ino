void bdbgPeriodicRequest() {
  if (millis() - bdbg_last_req >= BDBG_REQ_PERIOD_MS) {
    bdbg_last_req = millis();
    digitalWrite(BDBG_DIR_PIN, HIGH);
    while (Serial2.available()) Serial2.read();
    const uint8_t cmd[] = {0x55, 0xAA, 0x01};
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();
    digitalWrite(BDBG_DIR_PIN, LOW);
  }
}

void bdbgFeedByte(uint8_t b) {
  bdbg_buf[bdbg_idx] = b;
  if (bdbg_idx < sizeof(bdbg_buf) - 1) bdbg_idx++;
  bdbg_has_data = true;
  bdbg_reset_ts = millis();
}

void bdbgTryFinalizeFrame() {
  if (bdbg_has_data && millis() - bdbg_reset_ts >= METEO_FRAME_SETTLE_MS) {
    if (bdbg_idx == 10) {
      for (uint8_t i = 0; i < 10; i++) bdbg_raw[i] = bdbg_buf[i];
      uint32_t raw = ((uint32_t)bdbg_raw[6] << 24) | ((uint32_t)bdbg_raw[5] << 16) |
                     ((uint32_t)bdbg_raw[4] << 8)  | ((uint32_t)bdbg_raw[3]);
      radiation_uSvh = raw / 100.0f;
    } else {
      Serial.print("BDBG: bad length="); Serial.println(bdbg_idx);
    }
    bdbg_idx = 0; bdbg_has_data = false;
  }
}
