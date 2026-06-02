#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "pid.h"

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
float current_target_yaw = 0.0; // Lưu góc mục tiêu hiện tại cho PID

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit(); 
    
    Init_MPU(MPU_SDA_PIN, MPU_SCL_PIN); // Khởi tạo MPU6050

    grid[0][0] = 1;

    Serial.println("MAZE SOLVER START (Tang toc do di thang len 70)");
    Serial.printf("Kich thuoc Sa ban hien tai: X = 0->%d, Y = 0->%d\n", MAX_X, MAX_Y);
}

void loop() {
    updateAngle(); // Đọc góc liên tục từ MPU6050
    
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
        setMotors(0, 0);
        delay(1000);
        currentMillis = millis();

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

        // --- ĐÓNG NHÁNH CỤT (DEAD-END FILLING) ---
        // Nếu tất cả các hướng mở đều bị chặn, xe buộc phải quay lại đường cũ (bestScore >= 1000)
        // Ta đánh dấu vĩnh viễn ô ngõ cụt này thành vật cản (2) để xe không bao giờ rẽ vào lại nữa.
        if (bestScore >= 1000) {
            grid[currentX][currentY] = 2; 
        }

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
        driveWithHeading(70, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime >= 250) { 
            currentState = pendingTurn; 
            turnPhase = 0;
            actionStartTime = currentMillis;
            
            // Cập nhật góc mục tiêu dựa trên hướng rẽ
            // Đã sửa lại: Rẽ phải (cùng chiều kim đồng hồ) là góc ÂM, Rẽ trái là góc DƯƠNG
            if (pendingTurn == TURN_RIGHT) current_target_yaw = normalizeAngle(current_target_yaw - 70.0);
            else if (pendingTurn == TURN_LEFT) current_target_yaw = normalizeAngle(current_target_yaw + 70.0);
            else if (pendingTurn == TURN_AROUND) current_target_yaw = normalizeAngle(current_target_yaw + 180.0);
        }
        return;
    }

    // --- QUAY GÓC ---
    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        
        float error_val = calculateAngleError(current_target_yaw, current_angle);
        float error_abs = abs(error_val);
        
        // Chờ MPU quay đạt đến góc mục tiêu (nới lỏng sai số < 10 độ)
        if (error_abs < 10.0) {
            setMotors(0, 0); // Khóa cứng 2 bánh khi đạt góc chống trôi lố
            if (turnPhase == 0) {
                turnPhase = 1;
                actionStartTime = currentMillis; 
            } else if (currentMillis - actionStartTime >= 100) { // Đã ổn định góc trong 100ms
                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT) currentDir = (currentDir + 3) % 4;
                else currentDir = (currentDir + 2) % 4;

                currentState = PUSH_THROUGH; 
                actionStartTime = currentMillis;
            }
        } else {
            turnPhase = 0; // Nếu lệch lại thì reset cờ đếm thời gian
            
            // Khóa 1 bánh, rẽ 1 bánh tốc độ cao (80). Có xử lý giật lùi nhẹ nếu bị trớn quay lố góc.
            if (currentState == TURN_RIGHT) {
                if (error_val < 0) setMotors(80, 0); // Đang thiếu góc -> quay tiếp (trái chạy, phải khóa 0)
                else setMotors(-50, 0);              // Quay lố đà -> giật lùi bánh trái lại để sửa góc
            } else if (currentState == TURN_LEFT) {
                if (error_val > 0) setMotors(0, 80); // Đang thiếu góc -> quay tiếp (phải chạy, trái khóa 0)
                else setMotors(0, -50);              // Quay lố đà -> giật lùi bánh phải lại để sửa góc
            } else {
                // Quay đầu thì 2 bánh ngược chiều nhau tại chỗ
                if (error_val > 0) setMotors(-60, 60); 
                else setMotors(60, -60);
            }
        }
        return;
    }

    // --- VƯỢT THOÁT NGÃ TƯ ---
    if (currentState == PUSH_THROUGH) {
        driveWithHeading(70, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime > 150 && val != 0) {
            currentState = FOLLOW_LINE;
        } else if (currentMillis - actionStartTime > 600) { 
            currentState = FOLLOW_LINE;
        }
        return;
    }

    // --- LÙI VỀ NGÃ TƯ ---
    if (currentState == REVERSE_TO_NODE) {
        // Dùng lại PID để lùi. Do pidStraight đã được làm mềm, nó sẽ chỉ bù trừ nhẹ để đuôi xe không văng,
        // đồng thời khắc phục được lỗi động cơ vật lý bị lệch (kéo lùi không đều).
        driveWithHeading(-60, current_target_yaw, current_angle, pidStraight); 
        
        if (currentMillis - actionStartTime > 300 && val == 0) {
            setMotors(0, 0);
            delay(50); 
            currentState = FOLLOW_LINE; 
        }
        return;
    }

    // --- BÁM VẠCH / TRÁNH VẬT CẢN ---
    if (currentState == FOLLOW_LINE) {

        if (currentDistance > 0 && currentDistance <= 5) { 
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

                // Khóa cứng góc vật lý hiện tại làm mục tiêu mới.
                // Bộ PID sẽ giữ xe lùi thẳng tắp theo phương đang đứng, không bị vặn sườn.
                current_target_yaw = current_angle; 

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