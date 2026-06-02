#ifndef INDICATOR_H
#define INDICATOR_H

#include <Arduino.h>

void indicatorInit();
void indicatorUpdate();                  // Hàm duy trì trạng thái bằng millis (đặt trong loop)
void indicatorShowStripe(int count);     // Đổi màu đèn theo số đếm + Kêu bíp ngắn
void indicatorStartEndBlink();           // Bắt đầu chu trình chớp 8 lần + kêu còi
void indicatorClear();                   // Tắt toàn bộ đèn và còi
bool isIndicatorBusy();                  // Trả về true nếu đang trong quá trình chớp nháy

#endif