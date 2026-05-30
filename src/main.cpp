#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"

Master comm;

unsigned long lastI2CPoll = 0; 
long currentDistance = 999;

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
    TURN_RIGHT,
    TURN_LEFT,
    TURN_AROUND,  
    PUSH_THROUGH,
    FINISHED
};
State currentState = FOLLOW_LINE; 
State pendingTurn = TURN_RIGHT; 
unsigned long actionStartTime = 0;
int turnPhase = 0;

// Bộ lọc nhiễu vật cản siêu âm
int obstacleCount = 0; 

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit(); 

    grid[0][0] = 1;

    Serial.println("MAZE SOLVER START (Ban Chuan)");
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
    
    // --- BẮT NGÃ TƯ & SUY NGHĨ TỨC THÌ (KHÔNG DỪNG KHỰNG) ---
    if (currentState == FOLLOW_LINE && val == 0) {
        // Cập nhật la bàn toạ độ
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

        // TÍNH TOÁN HƯỚNG ĐI NGAY LẬP TỨC 
        int bestDir = -1;
        int bestScore = 9999;

        auto evaluateDirection = [&](int dir, int nx, int ny) {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y) return; 
            if (grid[nx][ny] == 2) return;                  

            int score = 0;
            // Phạt điểm nếu ô đó đã từng đi qua (để ưu tiên đường mới)
            if (grid[nx][ny] == 1) score += 1000; 

            if (dir == 0) score += 10;      // Ưu tiên đi Lên
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

        // Chốt hướng đi
        if (bestDir == currentDir) {
            // Đi thẳng: Vượt qua luôn, không cần tịnh tiến chờ rẽ
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        } else {
            // Rẽ: Chuyển sang đi mồi để tâm xe đè lên ngã tư
            currentState = NODE_ARRIVED; 
            actionStartTime = currentMillis;
            
            if (bestDir == (currentDir + 1) % 4) pendingTurn = TURN_RIGHT;
            else if (bestDir == (currentDir + 3) % 4) pendingTurn = TURN_LEFT;
            else pendingTurn = TURN_AROUND;
        }
        return;
    }

    // --- LẾT LÊN TÂM NGÃ TƯ KHI CẦN RẼ ---
    if (currentState == NODE_ARRIVED) {
        setMotors(80, 80); 
        // Đi tiếp 200ms để 2 bánh đè đúng tâm vạch ngang
        if (currentMillis - actionStartTime >= 200) { 
            currentState = pendingTurn; 
            turnPhase = 0;
            actionStartTime = currentMillis;
        }
        return;
    }

    // --- CÁCH XOAY GÓC TẠI NGÃ TƯ & QUAY ĐẦU (180 ĐỘ) ---
    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        if (currentState == TURN_RIGHT || currentState == TURN_AROUND) setMotors(100, -100);
        else setMotors(-100, 100);

        if (turnPhase == 0) {
            // Nhắm mắt vượt qua vạch ngang an toàn
            int blindTime = (currentState == TURN_AROUND) ? 400 : 150; 
            if (currentMillis - actionStartTime > blindTime) {
                turnPhase = 1;
            }
        } 
        else if (turnPhase == 1) {
            // Bất cứ mắt nào chạm vạch đen là phanh (khác nền trắng 31 và khác đen toàn bộ 0)
            if (val != 31 && val != 0) {
                // Phanh nghịch chiều để xe đứng yên không bị văng
                if (currentState == TURN_RIGHT || currentState == TURN_AROUND) setMotors(-60, 60); 
                else setMotors(60, -60);
                delay(50); 
                setMotors(0, 0);

                // Cập nhật la bàn
                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT) currentDir = (currentDir + 3) % 4;
                else if (currentState == TURN_AROUND) currentDir = (currentDir + 2) % 4;

                currentState = PUSH_THROUGH; 
                actionStartTime = currentMillis;
            }
        }
        return;
    }

    // --- VƯỢT THOÁT NGÃ TƯ MƯỢT MÀ ---
    if (currentState == PUSH_THROUGH) {
        setMotors(80, 80); 
        if (currentMillis - actionStartTime > 150 && val != 0) {
            currentState = FOLLOW_LINE;
        } else if (currentMillis - actionStartTime > 600) { // Safety timeout
            currentState = FOLLOW_LINE;
        }
        return;
    }

    // --- DÒ LINE BÌNH THƯỜNG TRÊN ĐOẠN THẲNG ---
    if (currentState == FOLLOW_LINE) {

        // --- LỌC NHIỄU VẬT CẢN (TRÁNH TƯỜNG ẢO) ---
        if (currentDistance > 0 && currentDistance <= 6) { 
            obstacleCount++;
            if (obstacleCount >= 3) { // 3 lần liên tiếp (30ms) mới tính là tường thật
                setMotors(0, 0);
                Serial.println("Gap TUONG o 6cm! Danh dau vao ban do & Quay dau!");
                
                int nx = currentX, ny = currentY;
                if (currentDir == 0) ny++;
                else if (currentDir == 1) nx++;
                else if (currentDir == 2) ny--;
                else if (currentDir == 3) nx--;
                
                if (nx >= 0 && nx <= MAX_X && ny >= 0 && ny <= MAX_Y) {
                    grid[nx][ny] = 2; // Đánh dấu mìn
                }

                // Cập nhật toạ độ ảo thành ô vật cản để khi quay đầu đi về, logic trừ toạ độ của ngã tư hoạt động chính xác
                currentX = nx;
                currentY = ny;

                currentState = TURN_AROUND;
                turnPhase = 0;
                actionStartTime = currentMillis;
                obstacleCount = 0; // Reset bộ đếm
                return;
            }
        } else {
            obstacleCount = 0;
        }

        // --- BỘ LUẬT CÂN BẰNG ĐỘNG CƠ TRÊN VẠCH ---
        switch (val) {
            case 27: case 17: setMotors(80, 80); break; // Cân bằng 2 bánh
            
            // Nhóm lệch trái
            case 19: setMotors(60, 80); break; 
            case 23: setMotors(40, 80); break; 
            case 7:  setMotors(20, 90); break; 
            case 15: setMotors(0, 100); break; 
            case 3:  setMotors(-20, 110); break;
            case 1:  setMotors(-40, 120); break; 
            
            // Nhóm lệch phải
            case 25: setMotors(80, 60); break; 
            case 29: setMotors(80, 40); break; 
            case 28: setMotors(90, 20); break; 
            case 30: setMotors(100, 0); break; 
            case 24: setMotors(110, -20); break; 
            case 16: setMotors(120, -40); break; 
            
            case 31: setMotors(60, 60); break; 
            default: setMotors(80, 80); break; 
        }
    }
}