#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"

Master comm;

unsigned long lastI2CPoll = 0; 
long currentDistance = 999;

// Đã mở rộng bản đồ để xe không bị sợ "tường ảo" và tự quay đầu bậy bạ
const int MAX_X = 6; 
const int MAX_Y = 3;
int grid[7][4] = {0}; 

int currentX = 0; 
int currentY = 0; 
int currentDir = 0; // 0: LÊN, 1: PHẢI, 2: XUỐNG, 3: TRÁI

// --- TRẠNG THÁI XE ---
enum State { 
    FOLLOW_LINE,
    NODE_ARRIVED,
    NODE_EVALUATE,
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,  
    PUSH_THROUGH,
    FINISHED
};
State currentState = NODE_EVALUATE; 
unsigned long actionStartTime = 0;
int turnPhase = 0;

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit(); 

    grid[0][0] = 1;
    actionStartTime = millis(); 

    Serial.println("MAZE SOLVER START");
    Serial.printf("Kich thuoc Sa ban hien tai: X = 0->%d, Y = 0->%d\n", MAX_X, MAX_Y);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentState == FINISHED) {
        setMotors(0, 0); 
        return;
    }
    if (currentMillis - lastI2CPoll < 10) return;
    lastI2CPoll = currentMillis;

    // --- ĐỌC I2C VÀ TÁCH BYTE ---
    uint8_t rx_data[2] = {255, 255}; 
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data); 
    
    if (rx_data[0] == 255 && rx_data[1] == 255) return; 

    currentDistance = (long)rx_data[0];     
    uint8_t val = (~rx_data[1]) & 0x1F;        
    if (currentState == FOLLOW_LINE && val == 0) {
        setMotors(90, 90); 
        delay(400);
        setMotors(0, 0); 
        
        if (currentDir == 0) currentY++;
        else if (currentDir == 1) currentX++;
        else if (currentDir == 2) currentY--;
        else if (currentDir == 3) currentX--;

        grid[currentX][currentY] = 1; 

        Serial.printf("\nDen nga tu: X=%d, Y=%d\n", currentX, currentY);

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

        
        int bestDir = -1;
        int bestScore = 9999;

        auto evaluateDirection = [&](int dir, int nx, int ny) {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y) return; 
            if (grid[nx][ny] == 2) return; // Nếu ô đó đã bị đánh dấu mìn thì bỏ qua                    

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

    // CÁCH XOAY GÓC TẠI NGÃ TƯ & QUAY ĐẦU (180 ĐỘ)
    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        if (currentState == TURN_RIGHT || currentState == TURN_AROUND) setMotors(110, -110);
        else setMotors(-110, 110);

        if (turnPhase == 0) {
            // Quay đầu cần văng xe mạnh hơn rẽ nên dùng 350ms, rẽ thường thì 100ms
            int blindTime = (currentState == TURN_AROUND) ? 350 : 100;
            if (currentMillis - actionStartTime > blindTime) {
                turnPhase = 1;
            }
        } 
        else if (turnPhase == 1) {
            // Đợi mắt dò line vớt được vạch
            if (val == 27 || val == 17 || val == 19 || val == 23) {
                if (currentState == TURN_RIGHT || currentState == TURN_AROUND) setMotors(-60, 60); else setMotors(60, -60);
                delay(60); 
                setMotors(0, 0);

                // Cập nhật la bàn tuỳ theo hướng vừa xoay
                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT) currentDir = (currentDir + 3) % 4;
                else if (currentState == TURN_AROUND) currentDir = (currentDir + 2) % 4;

                currentState = PUSH_THROUGH; 
                actionStartTime = currentMillis;
            }
        }
        return;
    }

    // VƯỢT THOÁT NGÃ TƯ
    if (currentState == PUSH_THROUGH) {
        setMotors(100, 100); 
        if (currentMillis - actionStartTime > 200 && val != 0) {
            currentState = FOLLOW_LINE;
        } else if (currentMillis - actionStartTime > 1000) {
            currentState = FOLLOW_LINE;
        }
        return;
    }

    // DÒ LINE BÌNH THƯỜNG TRÊN ĐOẠN THẲNG
    if (currentState == FOLLOW_LINE) {

        if (currentDistance > 0 && currentDistance <= 6) { 
            setMotors(0, 0);
            Serial.println("Gap vat can o 5cm! Quay dau 180 do ve ngã tư cũ!");
            
            // 1. Tìm toạ độ ô bị chặn phía trước
            int nx = currentX, ny = currentY;
            if (currentDir == 0) ny++;
            else if (currentDir == 1) nx++;
            else if (currentDir == 2) ny--;
            else if (currentDir == 3) nx--;
            
            if (nx >= 0 && nx <= MAX_X && ny >= 0 && ny <= MAX_Y) {
                grid[nx][ny] = 2; // Ghi nhớ đây là tường/vật cản vĩnh viễn
            }

            currentX = nx;
            currentY = ny;

            // 3. Ra lệnh quay đầu
            currentState = TURN_AROUND;
            turnPhase = 0;
            actionStartTime = currentMillis;
            return;
        }

        // --- CÁC NƯỚC CÂN BẰNG XE TRÊN VẠCH MÀU TRẮNG ---
        switch (val) {
            case 27: case 17: setMotors(85, 80); break; 
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
            default: setMotors(85, 80); break;
        }
    }
}