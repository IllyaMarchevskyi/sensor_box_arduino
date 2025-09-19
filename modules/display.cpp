#include <Arduino.h>
#include <TFT_eSPI.h>

// Moved from Globals.h (module-specific)
TFT_eSPI tft;
static uint32_t tft_update_timer = 0;

const int col1_x = 0, col1_x_value = 80, col2_x = 170, col2_x_value = 240, col3_x = 330, col3_x_value = 410, y_step = 30;
const int len_col = 10;
static float prev_send_arr[31] = {0};
const int len_prev_send_arr = sizeof(prev_send_arr)/sizeof(prev_send_arr[0]);

// Prototypes for local functions
void drawData();
void drawOnlyValue();

void initDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  drawData();
  // for (int i = 0; i < len_prev_send_arr; i++) prev_send_arr[i] = -100;
  // tft.setFreeFont(&FreeSans18pt7b); // ≈1.5x
  // tft.drawString("18pt ~ 1.5x", 10, 90);
}


void drawData() {
  // static float prev_send_arr[31] = {0};
  for (int i = 0; i < labels_len; i++) {
      int x = (i < len_col) ? col1_x : (i < len_col * 2) ? col2_x : col3_x;
      int y = (i < len_col) ? i * y_step : (i < len_col * 2) ? (i - len_col) * y_step: (i - len_col*2) * y_step;
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
  if (millis() - tft_update_timer <  1000) return;
  tft_update_timer += 1000;
  for (int i = 0; i < labels_len; i++) {
    if (send_arr[i] != prev_send_arr[i]) {
      int x = (i < len_col) ? col1_x_value : (i < len_col*2) ? col2_x_value: col3_x_value;
      int y = (i < len_col) ? i * y_step : (i < len_col * 2) ? (i - len_col) * y_step: (i - len_col*2) * y_step;
      tft.fillRect(x, y, 80, y_step, TFT_BLACK);
      tft.setCursor(x, y);
      if (send_arr[i] == -1) tft.print("#.##"); else tft.print(send_arr[i], 2);
      prev_send_arr[i] = send_arr[i];
    }
  }
}
