#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include <ESP32Servo.h>

// --- Cấu hình Cảm biến siêu âm HC-SR04 ---
#define TRIG_PIN 13
#define ECHO_PIN 12

// --- Cấu hình Servo Tay gắp (180 độ) ---
#define SERVO_PIN 38
#define GRIPPER_OPEN 0     // Góc mở tay gắp
#define GRIPPER_CLOSE 90   // Góc đóng tay gắp (kẹp vật)

Master comm;
Servo gripper; 

unsigned long lastI2CPoll = 0; 
unsigned long lastSonarPoll = 0;
long currentDistance = 999;

// Trạng thái xe có đang ngậm vật hay không
bool hasObject = false; 

// --- MÁY TRẠNG THÁI: CHỞ HÀNG ---
enum State { FOLLOW, UTURN, PICKING_UP, DROPPING_OFF, WAITING_CLEAR };
State currentState = FOLLOW;
unsigned long actionStartTime = 0;

// Biến lọc nhiễu mất line
int lostLineCount = 0;
static uint8_t last_valid_val = 27; 

// Hàm đo khoảng cách HC-SR04
long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); 
    if (duration == 0) return 999;
    return duration * 0.034 / 2;
}

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit();
    
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Khởi tạo Servo an toàn, không giật động cơ
    ESP32PWM::allocateTimer(3); 
    gripper.setPeriodHertz(50);
    gripper.attach(SERVO_PIN, 500, 2400);
    gripper.write(GRIPPER_OPEN); 

    Serial.println("KHOI DONG: Dung 3s gap vat -> Tha o vach -> Cho rut vat ra moi di tiep");
}

void loop() {
    unsigned long currentMillis = millis();

    // ==========================================
    // 1. ĐỌC CẢM BIẾN TRƯỚC TIÊN (Chạy liên tục)
    // ==========================================
    if (currentMillis - lastSonarPoll >= 50) {
        currentDistance = getDistance();
        lastSonarPoll = currentMillis;
    }

    if (currentMillis - lastI2CPoll < 10) return;
    lastI2CPoll = currentMillis;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return; 
    uint8_t val = raw_val & 0x1F; 

    Serial.printf("Khoang cach: %ld cm | Line: %d | Co vat: %d\n", currentDistance, val, hasObject);

    // ==========================================
    // 2. CÁC TRẠNG THÁI GẮP, THẢ, CHỜ ĐỢI
    // ==========================================
    if (currentState == PICKING_UP) {
        setMotors(0, 0); 
        gripper.write(GRIPPER_CLOSE); // BƠM LỆNH KẸP LIÊN TỤC TRONG 3S
        if (currentMillis - actionStartTime >= 3000) { 
            hasObject = true;
            currentState = FOLLOW;
            Serial.println("Gap xong 3s -> TIEP TUC DI TRUYEN!");
        }
        return; 
    }

    if (currentState == DROPPING_OFF) {
        setMotors(0, 0); 
        gripper.write(GRIPPER_OPEN); // BƠM LỆNH MỞ LIÊN TỤC TRONG 1S
        if (currentMillis - actionStartTime >= 1000) { 
            hasObject = false;
            currentState = WAITING_CLEAR; 
            Serial.println("Tha xong -> DUNG IM CHO NGUOI LAY VAT!");
        }
        return;
    }

    if (currentState == WAITING_CLEAR) {
        setMotors(0, 0); 
        gripper.write(GRIPPER_OPEN); // TIẾP TỤC GIỮ LỆNH MỞ CÀNG
        // Trông chừng khoảng cách, nếu > 15cm (hoặc 999) nghĩa là người dùng đã nhấc vật ra
        if (currentDistance > 15 || currentDistance == 999) { 
            currentState = FOLLOW; 
            Serial.println("Da rut vat ra -> TIEP TUC HANH TRINH!");
        }
        return;
    }

    // ==========================================
    // 3. XỬ LÝ QUAY ĐẦU TÌM VẠCH (U-TURN)
    // ==========================================
    if (currentState == UTURN) {
        setMotors(80, -80); 
        if (val != 31) { 
            currentState = FOLLOW; 
            lostLineCount = 0;
            Serial.println("Da bat lai duoc line -> DI TIEP!");
        }
        return;
    }

    // ==========================================
    // 4. LỌC NHIỄU MẤT LINE 
    // ==========================================
    if (val == 31) {
        lostLineCount++;
        if (lostLineCount >= 10) { 
            currentState = UTURN;
            lostLineCount = 0;
            Serial.println("Mat line That su -> QUAY DAU!");
        } else {
            val = last_valid_val; // Đắp trí nhớ vào để lướt qua nhiễu
        }
    } else {
        lostLineCount = 0; 
        last_valid_val = val;
    }

    // ==========================================
    // 5. PHÁT HIỆN VẬT CẢN (<= 10cm) -> GẮP
    // ==========================================
    if (!hasObject && currentDistance > 0 && currentDistance <= 10) {
        actionStartTime = currentMillis;
        currentState = PICKING_UP;
        Serial.println("Thay vat <= 10cm -> DUNG XE GAP VAT 3 GIAY!");
        return;
    }

    // ==========================================
    // 6. XỬ LÝ VẠCH NGANG (Mã 0) 
    // ==========================================
    if (val == 0) {
        if (hasObject) {
            actionStartTime = currentMillis;
            currentState = DROPPING_OFF;
            Serial.println("Den vach ngang -> THA VAT!");
            return; 
        } else {
            setMotors(70, 65); // Rỗng xe thì lướt qua luôn
        }
    } 
    else {
        // ==========================================
        // 7. BỘ LUẬT DÒ LINE THEO YÊU CẦU
        // ==========================================
        switch (val) {
            case 27: case 17:
                setMotors(70, 65); 
                break;
            case 19: setMotors(65, 70); break; 
            case 23: setMotors(45, 70); break; 
            case 7:  setMotors(20, 95); break; 
            case 15: setMotors(-30, 105); break; 
            case 3:  setMotors(-50, 125); break;
            case 1:  setMotors(-70, 135); break; 

            case 25: setMotors(75, 60); break; 
            case 29: setMotors(75, 40); break; 
            case 28: setMotors(100, 15); break; 
            case 30: setMotors(110, -35); break; 
            case 24: setMotors(130, -55); break; 
            case 16: setMotors(140, -75); break; 
            
            default:
                setMotors(70, 65); 
                break;
        }
    }
}