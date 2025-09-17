

void meteoFeedByte(uint8_t b) {
  Serial.println("Start Meteo");
  meteo_frame_buf[meteo_idx] = b;
  if (meteo_idx < sizeof(meteo_frame_buf) - 1) meteo_idx++;
  meteo_has_frame = true;
  meteo_reset_ts  = millis();
}

void meteoTryFinalizeFrame() {
  if (meteo_has_frame && millis() - meteo_reset_ts >= METEO_FRAME_SETTLE_MS) {
    if (meteo_idx == 17) {
      for (uint8_t i = 0; i < 21; i++) meteo_raw[i] = meteo_frame_buf[i];
      meteoDecodeToValues();
    } else {
      Serial.print("METEO: bad length="); Serial.println(meteo_idx);
    }
    meteo_idx = 0; meteo_has_frame = false;
    Serial.println("Finish Meteo");
  }
}

void meteoDecodeToValues() {
  uint16_t wind_dir_raw = ((meteo_raw[3] >> 4) << 8) | meteo_raw[2];
  meteo_dec[0] = (uint8_t)wind_dir_raw;

  int16_t temp_raw = ((meteo_raw[3] & 0x0F) << 8) | meteo_raw[4];
  meteo_dec[1] = (temp_raw - 400) / 10.0;

  meteo_dec[2] = meteo_raw[5];

  uint16_t wind_speed_raw = meteo_raw[6];
  meteo_dec[3] = ((float)wind_speed_raw / 8.0f) * 1.12f;

  meteo_dec[4] = meteo_raw[7] * 1.12f;
  meteo_dec[5] = ((meteo_raw[8] & 0x0F) << 8) | meteo_raw[9];
  meteo_dec[6] = ((meteo_raw[10] & 0x0F) << 8) | meteo_raw[11];

  uint32_t light = ((uint32_t)meteo_raw[12] << 16) | ((uint32_t)meteo_raw[13] << 8) | ((uint32_t)meteo_raw[14]);
  meteo_dec[7] = light / 10.0;

  uint32_t p = ((uint32_t)meteo_raw[17] << 16) | ((uint32_t)meteo_raw[18] << 8) | ((uint32_t)meteo_raw[19]);
  meteo_dec[8] = (double)p / 100.0;
}