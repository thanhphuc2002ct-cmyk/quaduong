#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"

// --- Cấu hình Cảm biến siêu âm HC-SR04 ---
#define TRIG_PIN 13
#define ECHO_PIN 12

Master comm;

unsigned long lastI2CPoll = 0; 
unsigned long lastSonarPoll = 0;
long currentDistance = 999;


// const int MAX_X = 8; 
// const int MAX_Y = 4;
// int grid[9][5] = {0}; 


const int MAX_X = 5; 
const int MAX_Y = 2;
int grid[6][3] = {0}; 


int currentX = 0; 
int currentY = 0; 

// Các hướng la bàn (0: LÊN, 1: PHẢI, 2: XUỐNG, 3: TRÁI)
int currentDir = 0; 

// --- TRẠNG THÁI XE ---
enum State { 
    FOLLOW_LINE,       // Đi thẳng bám vạch
    NODE_ARRIVED,      // Vừa chạm ngã tư
    NODE_EVALUATE,     // Đứng im tại ngã tư để quét mìn & ra quyết định
    TURN_RIGHT,        // Xoay phải 90 độ tại ngã tư
    TURN_LEFT,         // Xoay trái 90 độ tại ngã tư
    PUSH_THROUGH,      // Bơm ga vượt thoát khỏi vùng vạch đen ngã tư
    FINISHED           // Đã đến đích cuối cùng
};
State currentState = NODE_EVALUATE; 
unsigned long actionStartTime = 0;
int turnPhase = 0;

// Hàm đo khoảng cách
long getDistance() {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
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

    // Xuất phát từ (0,0), đánh dấu luôn ô này là đã đi qua (1)
    grid[0][0] = 1;
    actionStartTime = millis(); 

    Serial.println("MAZE SOLVER START: AI 2D ARRAY MAPPING");
    Serial.printf("Kich thuoc Sa ban hien tai: X = 0->%d, Y = 0->%d\n", MAX_X, MAX_Y);
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. Quét Radar (Siêu âm)
    if (currentMillis - lastSonarPoll >= 30) {
        long newDist = getDistance();
        if (newDist >= 0 && newDist < 400) currentDistance = newDist;
        else if (newDist == 999) currentDistance = 999;
        lastSonarPoll = currentMillis;
    }

    if (currentState == FINISHED) {
        setMotors(0, 0); 
        return;
    }

    if (currentMillis - lastI2CPoll < 10) return;
    lastI2CPoll = currentMillis;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return; 
    uint8_t val = raw_val & 0x1F; 

    // CÁCH DI CHUYỂN QUA CÁC NGÃ TƯ & CẬP NHẬT MẢNG
    if (currentState == FOLLOW_LINE && val == 0) {
        setMotors(0, 0); 
        
        if (currentDir == 0) currentY++;
        else if (currentDir == 1) currentX++;
        else if (currentDir == 2) currentY--;
        else if (currentDir == 3) currentX--;

        grid[currentX][currentY] = 1; 

        Serial.printf("\nDen nga tu: X=%d, Y=%d\n", currentX, currentY);

        // Kiểm tra đích đến dựa trên biến MAX động
        if (currentX == MAX_X && currentY == MAX_Y) {
            currentState = FINISHED;
            Serial.printf("🏆 ĐÃ TỚI ĐÍCH (%d,%d)! HOÀN THÀNH BẢN ĐỒ!\n", MAX_X, MAX_Y);
            return;
        }

        currentState = NODE_EVALUATE;
        actionStartTime = currentMillis;
        return;
    }

    // BỘ NÃO SUY NGHĨ TẠI NGÃ TƯ (NODE_EVALUATE)
    if (currentState == NODE_EVALUATE) {
        setMotors(0, 0); 
        
        if (currentMillis - actionStartTime < 1000) return;

        // BƯỚC 1: KIỂM TRA MÌN TRƯỚC MẶT & ĐÁNH DẤU VÀO MẢNG
        if (currentDistance > 0 && currentDistance <= 30) {
            int fx = currentX, fy = currentY;
            if (currentDir == 0) fy++;
            else if (currentDir == 1) fx++;
            else if (currentDir == 2) fy--;
            else if (currentDir == 3) fx--;

            if (fx >= 0 && fx <= MAX_X && fy >= 0 && fy <= MAX_Y) {
                grid[fx][fy] = 2; 
                Serial.printf("[-] PHAT HIEN MIN TAI: X=%d, Y=%d. Da cap nhat ban do!\n", fx, fy);
            }
        }

        // BƯỚC 2: AI TÍNH TOÁN ĐƯỜNG ĐI
        int bestDir = -1;
        int bestScore = 9999;

        auto evaluateDirection = [&](int dir, int nx, int ny) {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y) return; 
            if (grid[nx][ny] == 2) return;                    

            int score = 0;
            if (grid[nx][ny] == 1) score += 1000; 

            if (dir == 0) score += 10;      // Lên
            else if (dir == 1) score += 20; // Phải
            else if (dir == 3) score += 30; // Trái
            else if (dir == 2) score += 40; // Lùi

            if (score < bestScore) {
                bestScore = score;
                bestDir = dir;
            }
        };

        evaluateDirection(0, currentX, currentY + 1); 
        evaluateDirection(1, currentX + 1, currentY); 
        evaluateDirection(2, currentX, currentY - 1); 
        evaluateDirection(3, currentX - 1, currentY); 

        // BƯỚC 3: THỰC THI QUYẾT ĐỊNH
        if (bestDir == currentDir) {
            Serial.println("Quyen dinh: DI TIEP (Khong can xoay)");
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        } else if (bestDir == (currentDir + 1) % 4) {
            Serial.println("[AI] Quyen dinh: RE PHAI");
            currentState = TURN_RIGHT; turnPhase = 0; actionStartTime = currentMillis;
        } else if (bestDir == (currentDir + 3) % 4) {
            Serial.println("[AI] Quyen dinh: RE TRAI");
            currentState = TURN_LEFT; turnPhase = 0; actionStartTime = currentMillis;
        } else {
            Serial.println("[AI] Ket cut! Xoay mui de lui lai.");
            currentState = TURN_RIGHT; turnPhase = 0; actionStartTime = currentMillis;
        }
        return;
    }

    // CÁCH XOAY GÓC TẠI NGÃ TƯ
    if (currentState == TURN_RIGHT || currentState == TURN_LEFT) {
        if (currentState == TURN_RIGHT) setMotors(110, -110);
        else setMotors(-110, 110);

        if (turnPhase == 0) {
            if (currentMillis - actionStartTime > 100) {
                turnPhase = 1;
            }
        } 
        else if (turnPhase == 1) {
            if (val == 27 || val == 17 || val == 19 || val == 23) {
                if (currentState == TURN_RIGHT) setMotors(-60, 60); else setMotors(60, -60);
                delay(60); 
                setMotors(0, 0);

                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else currentDir = (currentDir + 3) % 4;

                currentState = NODE_EVALUATE; 
                actionStartTime = currentMillis;
            }
        }
        return;
    }

    // VƯỢT THOÁT NGÃ TƯ
    if (currentState == PUSH_THROUGH) {
        setMotors(85, 85); 
        if (currentMillis - actionStartTime > 100) {
            currentState = FOLLOW_LINE;
        }
        return;
    }

    // DÒ LINE BÌNH THƯỜNG TRÊN ĐOẠN THẲNG
    if (currentState == FOLLOW_LINE) {
        switch (val) {
            case 27: case 17: setMotors(80, 80); break;
            case 19: setMotors(75, 85); break; 
            case 23: setMotors(55, 85); break; 
            case 7:  setMotors(30, 100); break; 
            case 15: setMotors(-10, 110); break; 
            case 3:  setMotors(-30, 120); break;
            case 1:  setMotors(-50, 130); break; 
            case 25: setMotors(85, 75); break; 
            case 29: setMotors(85, 55); break; 
            case 28: setMotors(100, 30); break; 
            case 30: setMotors(110, -10); break; 
            case 24: setMotors(120, -30); break; 
            case 16: setMotors(130, -50); break; 
            case 31: setMotors(60, 60); break; 
            default: setMotors(80, 80); break;
        }
    }
}