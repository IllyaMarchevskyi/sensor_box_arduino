#include <Arduino.h>
#include <TFT_eSPI.h>

// Moved from Globals.h (module-specific)
TFT_eSPI tft;

// Time guard API from utils
bool time_guard_allow(const char* key, uint32_t interval_ms);

// Forward declaration from unique_id.ino
String uniqueIdString();

const int col1_x = 0, col1_x_value = 80, col2_x = 170, col2_x_value = 240, col3_x = 330, col3_x_value = 410, y_step = 30;
const int len_col = 10;
static float prev_send_arr[31] = {0};
const int len_prev_send_arr = sizeof(prev_send_arr)/sizeof(prev_send_arr[0]);

// Prototypes for local functions
void drawData();
void drawOnlyValue();
static void drawFooter();

void initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  drawData();
  drawFooter();
  // for (int i = 0; i < len_prev_send_arr; i++) prev_send_arr[i] = -100;
  // tft.setFreeFont(&FreeSans18pt7b); // ≈1.5x
  // tft.drawString("18pt ~ 1.5x", 10, 90);
}


void drawData() {
  // static float prev_send_arr[31] = {0};
  int x;
  int y;
  for (int i = 0; i < labels_len; i++) {
      x = (i < len_col) ? col1_x : (i < len_col * 2) ? col2_x : col3_x;
      y = (i < len_col) ? i * y_step : (i < len_col * 2) ? (i - len_col) * y_step: (i - len_col*2) * y_step;
      Serial.print("X: "); Serial.print(x); Serial.print("Y: "); Serial.println(y);
      tft.fillRect(x, y, 150, y_step, TFT_BLACK);
      char label[16]; sprintf(label, "%s: ", labels[i]);
      tft.setCursor(x, y); tft.print(label);
      x = (i < len_col) ? col1_x_value : (i < len_col*2) ? col2_x_value: col3_x_value;
      tft.setCursor(x, y);
      if (send_arr[i] == -1) tft.print("#.##"); else tft.print(send_arr[i], 2);
      prev_send_arr[i] = send_arr[i];
  }

}

void drawOnlyValue() {
  int x;
  int y;
  if (!time_guard_allow("draw/update", DRAW_MONITORING)) return;
  for (int i = 0; i < labels_len; i++) {
    if (send_arr[i] != prev_send_arr[i]) {
      x = (i < len_col) ? col1_x_value : (i < len_col*2) ? col2_x_value: col3_x_value;
      y = (i < len_col) ? i * y_step : (i < len_col * 2) ? (i - len_col) * y_step: (i - len_col*2) * y_step;
      tft.fillRect(x, y, 80, y_step, TFT_BLACK);
      tft.setCursor(x, y);
      if (send_arr[i] == -1) tft.print("#.##"); else tft.print(send_arr[i], 2);
      prev_send_arr[i] = send_arr[i];
    }
  }
  // Keep footer visible in case of screen updates
}

// Draw bottom-left HEX ID from EEPROM and bottom-right VERSION
static void drawFooter() {
  const int h = tft.height();
  const int w = tft.width();
  const int bar_h = 20; // enough for text size 2

  // Clear bottom bar
  tft.fillRect(0, h - bar_h, w, bar_h, TFT_BLACK);

  // Left bottom: label + HEX ID (as MAC-like)
  tft.setTextDatum(BL_DATUM);
  String hexId = uniqueIdString();
  tft.drawString(String("MAC: ") + hexId, 2, h - 1);

  // Right bottom: VERSION from Config.h
  tft.setTextDatum(BR_DATUM);
#ifdef VERSION
  tft.drawString(String("v") + VERSION, w - 2, h - 1);
#else
  tft.drawString("v?", w - 2, h - 1);
#endif
  // Reset datum to default for other drawings
  tft.setTextDatum(TL_DATUM);
}
