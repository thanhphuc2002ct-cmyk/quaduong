#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"

Master comm; 

uint16_t stripe_count = 0;
uint8_t last_val = 255;
bool saw_mark_27 = 0; 
bool is_stopped = 0;

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit();
}

void loop() {
    if (is_stopped == 1) return;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);

    if (raw_val == 255) {
        delay(10);
        return; 
    }

    uint8_t val = raw_val & 0x1F; 

    // 1. Nhận diện vạch nhô ra (27)
    if (val == 27) {
        saw_mark_27 = 1; 
    } 
    // Ra nền trắng hoàn toàn (31) -> Reset cờ
    else if (val == 31) {
        saw_mark_27 = 0; 
    }

    // 2. Gặp vạch ngang full (0)
    if (val == 0 && last_val != 0) { 
        // Ép dập xung bằng đúng hàm của thư viện để không lỗi Timer
        setMotors(0, 0); 
        
        if (saw_mark_27 == 1) {
            Serial.println("NGUNG 5 GIAY: Da thay vach nho ra 27 roi moi toi 0");
            delay(10000); 
            saw_mark_27 = 0; 
        } else {
            stripe_count++;
            Serial.printf("Dem vach soc: %d (Ngung 3s)\n", stripe_count);
            delay(3000); 
        }
        
        // Cấp xung chạy tiếp vọt qua vạch với đúng tốc độ ban đầu
        setMotors(45, 40);
        delay(400); 
    } 
    // 3. Mọi trường hợp khác -> Bơm ga chạy thẳng
    else if (val != 0) {
        setMotors(45, 40);
    }

    last_val = val;
    delay(10);
}  