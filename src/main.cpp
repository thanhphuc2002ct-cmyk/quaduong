#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"

Master comm;

unsigned long lastI2CPoll = 0; 
long currentDistance = 999;

const int MAX_X = 5; 
const int MAX_Y = 3;
int grid[6][4] = {0}; 

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
    REVERSE_TO_NODE,
    FINISHED
};
State currentState = FOLLOW_LINE; 
State pendingTurn = TURN_RIGHT; 
unsigned long actionStartTime = 0;
int turnPhase = 0;

int obstacleCount = 0; 

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit(); 

    grid[0][0] = 1;

    Serial.println("MAZE SOLVER START (Tang toc do di thang len 70)");
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

    uint8_t rx_data[2] = {255, 255}; 
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data); 
    
    if (rx_data[0] == 255 && rx_data[1] == 255) return; 

    currentDistance = (long)rx_data[0];     
    uint8_t val = (~rx_data[1]) & 0x1F;        
    
    // --- XỬ LÝ NGÃ TƯ ---
    if (currentState == FOLLOW_LINE && val == 0) {
        if (currentDir == 0) currentY++;
        else if (currentDir == 1) currentX++;
        else if (currentDir == 2) currentY--;
        else if (currentDir == 3) currentX--;

        grid[currentX][currentY] = 1; 

        Serial.printf("\nDen nga tu: X=%d, Y=%d\n", currentX, currentY);

        if (currentX == MAX_X && currentY == MAX_Y) {
            currentState = FINISHED;
            Serial.printf("ĐÃ TỚI ĐÍCH (%d,%d)\n", MAX_X, MAX_Y);
            return;
        }

        int bestDir = -1;
        int bestScore = 9999;

        auto evaluateDirection = [&](int dir, int nx, int ny) {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y) return; 
            if (grid[nx][ny] == 2) return; 

            int score = 0;
            if (grid[nx][ny] == 1) score += 1000; 

            if (dir == 0) score += 10;      
            else if (dir == 1) score += 20; 
            else if (dir == 3) score += 30; 
            else if (dir == 2) score += 40; 

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
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        } else {
            currentState = NODE_ARRIVED; 
            actionStartTime = currentMillis;
            
            if (bestDir == (currentDir + 1) % 4) pendingTurn = TURN_RIGHT;
            else if (bestDir == (currentDir + 3) % 4) pendingTurn = TURN_LEFT;
            else pendingTurn = TURN_AROUND;
        }
        return;
    }

    // --- TIẾN LÊN TÂM NGÃ TƯ ---
    if (currentState == NODE_ARRIVED) {
        setMotors(70, 70); // TĂNG LÊN 70
        if (currentMillis - actionStartTime >= 250) { 
            currentState = pendingTurn; 
            turnPhase = 0;
            actionStartTime = currentMillis;
        }
        return;
    }

    // --- QUAY GÓC ---
    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        
        if (currentState == TURN_RIGHT) setMotors(80, 0); 
        else if (currentState == TURN_LEFT) setMotors(0, 80); 
        else setMotors(-80, 80); 

        if (currentState == TURN_RIGHT || currentState == TURN_LEFT) {
            if (currentMillis - actionStartTime >= 800) {
                if (currentState == TURN_RIGHT) setMotors(-60, 0); 
                else setMotors(0, -60);
                delay(60); 
                setMotors(0, 0);

                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else currentDir = (currentDir + 3) % 4;

                currentState = PUSH_THROUGH; 
                actionStartTime = currentMillis;
            }
        } else {
            if (turnPhase == 0) {
                if (currentMillis - actionStartTime > 400) turnPhase = 1;
            } 
            else if (turnPhase == 1) {
                if (val != 31 && val != 0) {
                    setMotors(60, -60); 
                    delay(60); 
                    setMotors(0, 0);
                    currentDir = (currentDir + 2) % 4;
                    currentState = PUSH_THROUGH; 
                    actionStartTime = currentMillis;
                }
            }
        }
        return;
    }

    // --- VƯỢT THOÁT NGÃ TƯ ---
    if (currentState == PUSH_THROUGH) {
        setMotors(70, 70); // TĂNG LÊN 70
        if (currentMillis - actionStartTime > 150 && val != 0) {
            currentState = FOLLOW_LINE;
        } else if (currentMillis - actionStartTime > 600) { 
            currentState = FOLLOW_LINE;
        }
        return;
    }

    // --- LÙI VỀ NGÃ TƯ ---
    if (currentState == REVERSE_TO_NODE) {
        setMotors(-60, -60); 
        
        if (currentMillis - actionStartTime > 300 && val == 0) {
            setMotors(0, 0);
            delay(50); 
            currentState = FOLLOW_LINE; 
        }
        return;
    }

    // --- BÁM VẠCH / TRÁNH VẬT CẢN ---
    if (currentState == FOLLOW_LINE) {

        if (currentDistance > 0 && currentDistance <= 6) { 
            obstacleCount++;
            if (obstacleCount >= 3) { 
                setMotors(0, 0); 
                delay(1000); 

                int nx = currentX, ny = currentY;
                if (currentDir == 0) ny++;
                else if (currentDir == 1) nx++;
                else if (currentDir == 2) ny--;
                else if (currentDir == 3) nx--;
                
                if (nx >= 0 && nx <= MAX_X && ny >= 0 && ny <= MAX_Y) {
                    grid[nx][ny] = 2; 
                }

                if (currentDir == 0) currentY--;
                else if (currentDir == 1) currentX--;
                else if (currentDir == 2) currentY++;
                else if (currentDir == 3) currentX++;

                currentState = REVERSE_TO_NODE; 
                actionStartTime = millis(); 
                obstacleCount = 0; 
                return;
            }
        } else {
            obstacleCount = 0;
        }

        switch (val) {
            case 27: case 17: setMotors(70, 70); break; // TĂNG LÊN 70
            case 19: setMotors(45, 60); break; 
            case 23: setMotors(30, 60); break; 
            case 7:  setMotors(15, 70); break; 
            case 15: setMotors(0, 80); break; 
            case 3:  setMotors(-15, 90); break;
            case 1:  setMotors(-30, 100); break; 
            case 25: setMotors(60, 45); break; 
            case 29: setMotors(60, 30); break; 
            case 28: setMotors(70, 15); break; 
            case 30: setMotors(80, 0); break; 
            case 24: setMotors(90, -15); break; 
            case 16: setMotors(100, -30); break; 
            case 31: setMotors(40, 40); break; 
            default: setMotors(70, 70); break; // TĂNG LÊN 70
        }
    }
}