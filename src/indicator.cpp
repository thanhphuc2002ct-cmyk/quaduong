#include "indicator.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum IndState { IDLE, BEEPING_STRIPE, BLINKING_END };
IndState indState = IDLE;

unsigned long indPreviousMillis = 0;
int indBlinkCount = 0;
bool indLedState = false;

// Biến cho còi kêu nhiều nhịp
int targetBeeps = 0;
int currentBeepCount = 0;
bool isBeepOn = false;

void indicatorInit() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    pixels.begin();
    pixels.clear();
    pixels.show();
}

void indicatorShowStripe(int count) {
    pixels.clear();
    if (count == 1) pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    else if (count == 2) pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    else if (count == 3) pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    else if (count == 4) pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    else if (count == 5) pixels.setPixelColor(0, pixels.Color(128, 0, 128));
    else if (count == 6) pixels.setPixelColor(0, pixels.Color(0, 255, 255));
    pixels.show();

    // Thiết lập còi kêu số lần tương ứng với số vạch
    targetBeeps = count;
    currentBeepCount = 0;
    isBeepOn = true;
    digitalWrite(BUZZER_PIN, HIGH);
    
    indState = BEEPING_STRIPE;
    indPreviousMillis = millis();
}

void indicatorStartEndBlink() {
    indState = BLINKING_END;
    indBlinkCount = 0;
    indLedState = true;
    indPreviousMillis = millis();
    
    // Sáng đèn + kêu còi ngay nhịp đầu tiên
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    pixels.show();
    digitalWrite(BUZZER_PIN, HIGH); 
}

void indicatorClear() {
    pixels.clear();
    pixels.show();
    digitalWrite(BUZZER_PIN, LOW);
    indState = IDLE;
}

bool isIndicatorBusy() {
    return (indState == BLINKING_END);
}

void indicatorUpdate() {
    unsigned long currentMillis = millis();

    // 1. Kêu còi nhiều nhịp (Ví dụ: vạch 3 kêu 3 cái, mỗi cái 100ms kêu - 100ms tắt)
    if (indState == BEEPING_STRIPE) {
        if (currentMillis - indPreviousMillis >= 100) {
            indPreviousMillis = currentMillis;
            
            if (isBeepOn) {
                // Đang kêu thì tắt
                digitalWrite(BUZZER_PIN, LOW);
                isBeepOn = false;
                currentBeepCount++;
                
                // Nếu đã kêu đủ số lần thì chuyển về trạng thái IDLE
                if (currentBeepCount >= targetBeeps) {
                    indState = IDLE;
                }
            } else {
                // Đang tắt thì bật lại để kêu nhịp tiếp theo
                digitalWrite(BUZZER_PIN, HIGH);
                isBeepOn = true;
            }
        }
    }
    // 2. Chớp đèn trắng và kêu còi 8 lần
    else if (indState == BLINKING_END) {
        if (currentMillis - indPreviousMillis >= 500) {
            indPreviousMillis = currentMillis;
            indLedState = !indLedState; 

            if (indLedState) {
                pixels.setPixelColor(0, pixels.Color(255, 255, 255));
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                pixels.clear();
                digitalWrite(BUZZER_PIN, LOW);
            }
            pixels.show();
            
            indBlinkCount++;
            if (indBlinkCount >= 16) { 
                indicatorClear(); // Xong 8 vòng thì tự tắt
            }
        }
    }
}