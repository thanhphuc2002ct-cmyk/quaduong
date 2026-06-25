#include "modes.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "pid.h"

extern Master comm;
char remoteCmd = 0;

// Đọc khoảng cách từ cảm biến siêu âm HC-SR04 với timeout chống treo
long getSonarDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    
    static long lastValidDist = 999;
    static int errorCount = 0;

    if (duration == 0) {
        errorCount++;
        if (errorCount >= 2) {
            lastValidDist = 999;
        }
        return lastValidDist; 
    }

    errorCount = 0; 
    long dist = duration * 0.034 / 2;
    lastValidDist = dist;
    return dist;
}

// Giải mê cung bằng thuật toán ma trận kết hợp PID góc và PWM thuần túy
void modeMazeSolver(bool reset) {
    const int MAX_X = 5; 
    const int MAX_Y = 2;
    static int grid[6][3] = {0}; 
    static int currentX = 0; 
    static int currentY = 0; 
    static int currentDir = 0; 

    enum State { FOLLOW_LINE, NODE_ARRIVED, TURN_RIGHT, TURN_LEFT, TURN_AROUND, PUSH_THROUGH, REVERSE_TO_NODE, FINISHED };
    static State currentState = FOLLOW_LINE; 
    static State pendingTurn = TURN_RIGHT; 
    static unsigned long actionStartTime = 0;
    static int turnPhase = 0;
    static int obstacleCount = 0; 
    static float current_target_yaw = 0.0;
    static bool isInit = false;

    if (reset) {
        memset(grid, 0, sizeof(grid));
        currentX = 0; currentY = 0; currentDir = 0;
        currentState = FOLLOW_LINE; pendingTurn = TURN_RIGHT;
        actionStartTime = 0; turnPhase = 0; obstacleCount = 0;
        current_target_yaw = 0.0; current_angle = 0.0; isInit = false;
        return;
    }

    if (!isInit) {
        grid[0][0] = 1;
        isInit = true;
    }

    updateAngle(); 
    unsigned long currentMillis = millis();

    if (currentState == FINISHED) {
        setMotors(0, 0); 
        return;
    }
    
    if (currentMillis - actionStartTime < 10 && currentState != NODE_ARRIVED && currentState != TURN_RIGHT && currentState != TURN_LEFT && currentState != TURN_AROUND && currentState != PUSH_THROUGH && currentState != REVERSE_TO_NODE) {
        static unsigned long lastI2CPoll = 0;
        if (currentMillis - lastI2CPoll < 10) return;
        lastI2CPoll = currentMillis;
    }

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val); 
    if (raw_val == 255) return; 

    long currentDistance = getSonarDistance();     
    uint8_t val = raw_val & 0x1F;        
    
    if (currentState == FOLLOW_LINE && val == 31) {
        setMotors(0, 0);
        delay(500);
        updateAngle(); 
        currentMillis = millis();

        if (currentDir == 0) currentY++;
        else if (currentDir == 1) currentX++;
        else if (currentDir == 2) currentY--;
        else if (currentDir == 3) currentX--;

        currentX = constrain(currentX, 0, MAX_X);
        currentY = constrain(currentY, 0, MAX_Y);

        grid[currentX][currentY] = 1; 

        Serial.printf("\n--- DEN NGA TU: X=%d, Y=%d (Huong xe hien tai: %d) ---\n", currentX, currentY, currentDir);

        if (currentX == MAX_X && currentY == MAX_Y) {
            currentState = FINISHED;
            Serial.printf(">> HOAN THANH! Da toi dich (%d,%d)\n", MAX_X, MAX_Y);
            return;
        }

        int bestDir = -1;
        int bestScore = 9999;
        bool hasUnvisited = false;

        auto evaluateDirection = [&](int dir, int nx, int ny) {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y) return; 
            if (grid[nx][ny] == 2) return; 
            if (grid[nx][ny] == 0) hasUnvisited = true;
            
            int freeWays = 0;
            if (ny + 1 <= MAX_Y && grid[nx][ny + 1] != 2) freeWays++; 
            if (nx + 1 <= MAX_X && grid[nx + 1][ny] != 2) freeWays++; 
            if (ny - 1 >= 0 && grid[nx][ny - 1] != 2) freeWays++;     
            if (nx - 1 >= 0 && grid[nx - 1][ny] != 2) freeWays++;     
            
            if (freeWays <= 1 && !(nx == MAX_X && ny == MAX_Y)) {
                grid[nx][ny] = 2; 
                return; 
            }
            
            int score = 0;
            if (grid[nx][ny] == 1) score += 50; 
            
            score += (abs(MAX_X - nx) + abs(MAX_Y - ny)) * 10;
            
            int relDir = (dir - currentDir + 4) % 4;
            if (relDir == 0) score += 0;       
            else if (relDir == 1) score += 20; 
            else if (relDir == 3) score += 20; 
            else if (relDir == 2) score += 200; 

            if (score < bestScore) {
                bestScore = score;
                bestDir = dir;
            }
        };

        evaluateDirection(0, currentX, currentY + 1); 
        evaluateDirection(1, currentX + 1, currentY); 
        evaluateDirection(2, currentX, currentY - 1); 
        evaluateDirection(3, currentX - 1, currentY); 

        if (bestDir == -1) {
            Serial.println("-> MAZE VO NGHIEM! Tat ca cac huong deu bi chan.");
            currentState = FINISHED;
            return;
        }

        if (bestDir == currentDir) {
            Serial.println("-> Lua chon: DI THANG");
            current_target_yaw = round(current_angle / 90.0) * 90.0; 
            current_angle = current_target_yaw;
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        } else {
            currentState = NODE_ARRIVED; 
            actionStartTime = currentMillis;
            current_target_yaw = round(current_angle / 90.0) * 90.0; 
            current_angle = current_target_yaw;
            if (bestDir == (currentDir + 1) % 4) { pendingTurn = TURN_RIGHT; Serial.println("-> Lua chon: RE PHAI"); }
            else if (bestDir == (currentDir + 3) % 4) { pendingTurn = TURN_LEFT; Serial.println("-> Lua chon: RE TRAI"); }
            else { pendingTurn = TURN_AROUND; Serial.println("-> Lua chon: QUAY DAU (Ngo cut)"); }
        }
        return;
    }

if (currentState == NODE_ARRIVED) {
        int pushSpeed = 60 + (currentMillis - actionStartTime) / 4;
        if (pushSpeed > 75) pushSpeed = 75; 
        driveWithHeading(pushSpeed, current_target_yaw, current_angle, pidStraight);
        
        if (currentMillis - actionStartTime >= 250) {
            setMotors(0, 0); 
            delay(50);
            
            currentState = pendingTurn;

            turnPhase = 0;
            actionStartTime = millis(); // Cập nhật lại mốc thời gian sau delay
            
            if (pendingTurn == TURN_RIGHT) current_target_yaw = normalizeAngle(current_target_yaw - 50.0);
            else if (pendingTurn == TURN_LEFT) current_target_yaw = normalizeAngle(current_target_yaw + 50.0);
            else if (pendingTurn == TURN_AROUND) current_target_yaw = normalizeAngle(current_target_yaw + 180.0);
        }
        return;
    }

if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        float error_val = calculateAngleError(current_target_yaw, current_angle);
        float error_abs = abs(error_val);
        
        bool caughtLine = (error_abs < 40.0) && (val == 4 || val == 14 || val == 12 || val == 6 || val == 8 || val == 2);
        
        if (error_abs < 3.0 || caughtLine || turnPhase == 1) {
            setMotors(0, 0); 
            if (turnPhase == 0) {
                turnPhase = 1; // Khóa trạng thái phanh
                actionStartTime = currentMillis; 
            } else if (currentMillis - actionStartTime >= 100) { 
                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT) currentDir = (currentDir + 3) % 4;
                else currentDir = (currentDir + 2) % 4;
                
                currentState = FOLLOW_LINE; 
                actionStartTime = currentMillis;
            }
        } else {
            turnPhase = 0; 
            driveWithHeading(0, current_target_yaw, current_angle, pidTurn);
        }
        return;
    }


    if (currentState == PUSH_THROUGH) {
        int pushSpeed = 65 + (currentMillis - actionStartTime) / 3;
        if (pushSpeed > 85) pushSpeed = 85;
        driveWithHeading(pushSpeed, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime > 250 && val != 31) {
            currentState = FOLLOW_LINE;
        } else if (currentMillis - actionStartTime > 800) { // Timeout an toàn 800ms
            currentState = FOLLOW_LINE;
        }
        return;
    }
    if (currentState == REVERSE_TO_NODE) {
        driveWithHeading(-80, current_target_yaw, current_angle, pidStraight); 
        if (currentMillis - actionStartTime > 300 && val == 31) {
            setMotors(0, 0);
            current_angle = current_target_yaw;
            delay(50); 
            currentState = FOLLOW_LINE; 
        }
        return;
    }

    if (currentState == FOLLOW_LINE) {
        if (currentDistance > 0 && currentDistance <= 5) { 
            obstacleCount++;
            if (obstacleCount >= 3) { 
                setMotors(0, 0);
                delay(1000); 
                updateAngle(); 
                int nx = currentX, ny = currentY;
                if (currentDir == 0) ny++;
                else if (currentDir == 1) nx++;
                else if (currentDir == 2) ny--;
                else if (currentDir == 3) nx--;
                
                if (nx >= 0 && nx <= MAX_X && ny >= 0 && ny <= MAX_Y) {
                    grid[nx][ny] = 2; 
                    Serial.printf("!!! PHAT HIEN VAT CAN TAI: X=%d, Y=%d !!! Da cap nhat ban do.\n", nx, ny);
                }

                if (currentDir == 0) currentY--;
                else if (currentDir == 1) currentX--;
                else if (currentDir == 2) currentY++;
                else if (currentDir == 3) currentX++;
                Serial.println("-> Lui ve nga tu truoc do de tim duong khac...");
                current_target_yaw = round(current_angle / 90.0) * 90.0;
                currentState = REVERSE_TO_NODE; 

                // currentX = nx;
                // currentY = ny;
                // Serial.println("-> Quay dau 180 do ve nga tu truoc do de tim duong khac...");
                // currentState = TURN_AROUND;
                // turnPhase = 0;
                // current_target_yaw = normalizeAngle(round(current_angle / 90.0) * 90.0 + 180.0);
                
                // actionStartTime = millis(); 
                // obstacleCount = 0; 
                // return;
            }
        } else {
            obstacleCount = 0;
        }

switch (val) {
            case 4: case 14: setMotors(90, 90); break; 
            case 12: setMotors(86, 90); break; 
            case 8:  setMotors(78, 90); break; 
            case 24: setMotors(55, 95); break; 
            case 16: setMotors(35, 105); break; 
            case 28: setMotors(0, 110); break;
            case 30: setMotors(-30, 120); break; 
            case 6:  setMotors(90, 86); break; 
            case 2:  setMotors(90, 78); break; 
            case 3:  setMotors(95, 55); break; 
            case 1:  setMotors(105, 35); break; 
            case 7:  setMotors(110, 0); break; 
            case 15: setMotors(120, -30); break; 
            case 0:  driveWithHeading(80, current_target_yaw, current_angle, pidStraight); break; 
            default: driveWithHeading(90, current_target_yaw, current_angle, pidStraight); break; 
        }
    }
}


// Điều khiển xe bằng Remote thông qua MPU6050
void modeRemoteControl(bool reset) {
    enum RemoteState { IDLE, MOVING_FORWARD, MOVING_BACKWARD, TURNING };
    static RemoteState state = IDLE;
    static unsigned long actionStartTime = 0;
    static float target_yaw = 0.0;
    static bool isInit = false;

    if (reset) {
        state = IDLE;
        actionStartTime = 0;
        target_yaw = 0.0;
        isInit = false;
        remoteCmd = 0;
        return;
    }

    if (!isInit) {
        updateAngle();
        target_yaw = current_angle;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    switch (state) {
        case IDLE:
            setMotors(0, 0);
            if (remoteCmd == 'U') {
                state = MOVING_FORWARD;
                target_yaw = current_angle; 
                actionStartTime = currentMillis;
                remoteCmd = 0; 
            } else if (remoteCmd == 'D') {
                state = MOVING_BACKWARD;
                target_yaw = current_angle;
                actionStartTime = currentMillis;
                remoteCmd = 0;
            } else if (remoteCmd == 'L') {
                state = TURNING;
                target_yaw = normalizeAngle(current_angle + 90.0);
                remoteCmd = 0;
            } else if (remoteCmd == 'R') {
                state = TURNING;
                target_yaw = normalizeAngle(current_angle - 90.0);
                remoteCmd = 0;
            } else {
                remoteCmd = 0; 
            }
            break;

        case MOVING_FORWARD:
            driveWithHeading(120, target_yaw, current_angle, pidStraight);
            if (currentMillis - actionStartTime >= 2000) {
                state = IDLE;
            }
            break;

        case MOVING_BACKWARD:
            driveWithHeading(-120, target_yaw, current_angle, pidStraight);
            if (currentMillis - actionStartTime >= 2000) {
                state = IDLE;
            }
            break;

        case TURNING: {
            float error_val = calculateAngleError(target_yaw, current_angle);
            if (abs(error_val) < 3.0) {
                setMotors(0, 0);
                state = IDLE;
            } else {
                driveWithHeading(0, target_yaw, current_angle, pidTurn);
            }
            break;
        }
    }
}

void modeObstacleAvoidance(bool reset) {
    enum ObstacleState { FOLLOW, OBSTACLE };
    static ObstacleState state = FOLLOW;
    static unsigned long lastI2C = 0;
    static uint8_t lastVal = 4; 

    // Lấy biến siêu âm đã được quét sẵn từ main.cpp sang dùng
    extern long current_distance; 

    if (reset) {
        state = FOLLOW; lastI2C = 0; lastVal = 4;
        return;
    }
    
    unsigned long currentMillis = millis();

    if (current_distance > 0 && current_distance <= 18) {
        setMotors(0, 0);
        state = OBSTACLE;
        return; 
    } else {
        if (state == OBSTACLE) state = FOLLOW;
    }

    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t raw_val = 255;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return;

    uint8_t val = raw_val & 0x1F; 

    if (val == 0) val = lastVal;
    else lastVal = val;

    if (state == FOLLOW) {
        switch (val) {
            case 4: case 14: setMotors(90, 90); break; 
            case 12: setMotors(86, 90); break; 
            case 8:  setMotors(78, 90); break; 
            case 24: setMotors(55, 95); break; 
            case 16: setMotors(0, 125); break; 
            case 28: setMotors(0, 110); break;
            case 30: setMotors(0, 120); break; 
            case 6:  setMotors(90, 86); break; 
            case 2:  setMotors(90, 78); break; 
            case 3:  setMotors(95, 55); break; 
            case 1:  setMotors(125, 0); break; 
            case 7:  setMotors(110, 0); break; 
            case 15: setMotors(120, 0); break; 
            case 31: setMotors(80, 80); break; 
            default: setMotors(90, 90); break; 
        }
    }
}

// Điều khiển cơ cấu servo kẹp nhả vật thể dựa trên tín hiệu siêu âm
void modePickAndDrop(bool reset) {
    enum PickState { FOLLOW, PICKING_UP, TURN_RIGHT_DROP, DROPPING_ACTION, TURN_LEFT_BACK };
    static PickState state = FOLLOW;
    static unsigned long lastI2C = 0; 
    static bool hasObject = false;
    static unsigned long actionTime = 0;
    static uint8_t lastVal = 4; 
    static int detectCount = 0;
    static float drop_target_yaw = 0.0;
    static int turnPhase = 0;
    static Servo gripper;
    static bool isInit = false;

    if (reset) {
        state = FOLLOW; lastI2C = 0; hasObject = false; actionTime = 0;
        lastVal = 4; detectCount = 0; drop_target_yaw = 0.0; turnPhase = 0;
        isInit = false;
        return;
    }

    if (!isInit) {
        ESP32PWM::allocateTimer(0);
        gripper.setPeriodHertz(50);
        gripper.attach(48, 500, 2400); 
        gripper.write(0); 
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (state == TURN_RIGHT_DROP || state == TURN_LEFT_BACK) {
        float error_val = calculateAngleError(drop_target_yaw, current_angle);
        if (abs(error_val) < 10.0) {
            setMotors(0, 0);
            if (turnPhase == 0) {
                turnPhase = 1;
                actionTime = currentMillis;
            } else if (currentMillis - actionTime >= 100) {
                if (state == TURN_RIGHT_DROP) {
                    state = DROPPING_ACTION;
                    actionTime = currentMillis;
                } else {
                    state = FOLLOW;
                }
            }
        } else {
            turnPhase = 0;
            if (state == TURN_RIGHT_DROP) {
                if (error_val < 0) setMotors(80, 0); else setMotors(-50, 0);
            } else {
                if (error_val > 0) setMotors(0, 80); else setMotors(0, -50);
            }
        }
        return;
    }

    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t raw_val = 255;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return;

    long distance = getSonarDistance();
    uint8_t val = raw_val & 0x1F;

    if (state == PICKING_UP) {
        setMotors(0, 0);
        gripper.write(90); 
        if (currentMillis - actionTime >= 3000) { 
            hasObject = true;
            state = FOLLOW;
        }
        return;
    }

    if (state == DROPPING_ACTION) {
        setMotors(0, 0); 
        gripper.write(0);
        if (currentMillis - actionTime >= 1000) { 
            hasObject = false;
            drop_target_yaw = normalizeAngle(current_angle + 90.0);
            state = TURN_LEFT_BACK;
            turnPhase = 0;
        }
        return;
    }

    if (val == 0) val = lastVal;
    else lastVal = val;

    if (!hasObject && distance > 0 && distance <= 10) {
        detectCount++;
        if (detectCount >= 3) {
            actionTime = currentMillis;
            state = PICKING_UP;
            detectCount = 0; 
            return;
        }
    } else {
        detectCount = 0;
    }

    if (val == 31) {
        if (hasObject) {
            drop_target_yaw = normalizeAngle(current_angle - 90.0);
            state = TURN_RIGHT_DROP;
            turnPhase = 0;
            return;
        } else {
            setMotors(70, 65);
        }
    } else {
switch (val) {
                case 4: case 14: case 31: setMotors(80, 80); break;
                case 12: setMotors(75, 80); break; 
                case 8:  setMotors(55, 80); break; 
                case 24: setMotors(30, 105); break; 
                case 16: setMotors(-40, 115); break;
                case 28: setMotors(-60, 135); break;
                case 30: setMotors(-80, 145); break;
                case 6:  setMotors(85, 70); break; 
                case 2:  setMotors(85, 50); break; 
                case 3:  setMotors(110, 25); break; 
                case 1:  setMotors(120, -45); break; 
                case 7:  setMotors(140, -65); break; 
                case 15: setMotors(150, -85); break; 
                default: setMotors(80, 80); break;
            }
    }
}
// Đếm vạch ngang giao lộ và phản hồi tín hiệu âm thanh kết hợp chớp LED đa sắc
const int CROSS_LED_PIN = 38;
const int CROSS_NUMPIXELS = 8;
Adafruit_NeoPixel pixels(CROSS_NUMPIXELS, CROSS_LED_PIN, NEO_GRB + NEO_KHZ800);

void initCrossroadLED() {
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();
}

void triggerModeChangeSequence() {
    for (int i = 0; i < 8; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show(); delay(150);
    pixels.clear(); pixels.show();
}

void triggerStartSequence() {
    setMotors(0, 0);
    for (int i = 0; i < 8; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show(); delay(150);
    pixels.clear(); pixels.show(); delay(150);
    for (int i = 0; i < 8; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show(); delay(150);
    pixels.clear(); pixels.show(); delay(150);
    for (int i = 0; i < 8; i++) pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    pixels.show(); delay(250);
    pixels.clear(); pixels.show(); delay(150);
    
    extern unsigned long prev_time;
    prev_time = micros(); // Xóa tích lũy thời gian ảo do delay gây ra
}

void modeCrossroad(bool reset) {
    enum IndState { IDLE, IND_BLINKING_END };
    enum RobotState { RUNNING, PAUSING_STRIPE, PAUSING_END, BLINKING_END, CROSSING_LINE };
    
    static IndState indState = IDLE;
    static RobotState state = RUNNING;
    static unsigned long indPrevMillis = 0;
    static int blinkCount = 0;
    static bool ledState = false;
    static uint16_t stripeCount = 0;
    static uint8_t lastVal = 255;
    static unsigned long prevMillis = 0;
    static unsigned long lastI2C = 0;
    static float cross_target_yaw = 0.0;
    static int phase = 0; 
    static int dirMode = 0; 
    static bool readyToStart = false; 
    static bool isInit = false; // Thêm cờ khởi tạo góc chuẩn

    if (reset) {
        indState = IDLE; state = RUNNING; indPrevMillis = 0; blinkCount = 0;
        ledState = false; stripeCount = 0; lastVal = 255;
        prevMillis = 0; lastI2C = 0; 
        phase = 0; dirMode = 0; readyToStart = false;
        isInit = false; // Reset lại cờ khi reset mode
        
        updateAngle();
        cross_target_yaw = current_angle; 
        
        pixels.clear();
        pixels.show(); 
        return;
    }

    // LẤY GÓC HIỆN TẠI LÀM TIÊU CHUẨN NGAY KHI VÀO MODE
    if (!isInit) {
        updateAngle();
        cross_target_yaw = current_angle;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (indState == IND_BLINKING_END) {
        if (currentMillis - indPrevMillis >= 500) {
            indPrevMillis = currentMillis;
            ledState = !ledState; 
            if (ledState) {
                for(int i = 0; i < CROSS_NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
            } else {
                pixels.clear();
            }
            pixels.show();
            blinkCount++;
            if (blinkCount >= 10) { 
                pixels.clear();
                pixels.show();
                indState = IDLE;
            }
        }
    }

    switch (state) {
        case PAUSING_STRIPE:
            if (currentMillis - prevMillis >= 2000) {
                state = CROSSING_LINE;
            }
            return;
            
        case PAUSING_END:
            if (currentMillis - prevMillis >= 10000) {
                state = CROSSING_LINE;
                prevMillis = currentMillis;
            }
            return;
            
        case BLINKING_END:
            setMotors(0, 0);
            if (indState != IND_BLINKING_END) {
                state = PAUSING_END;
                prevMillis = currentMillis + 9999999; 
            }
            return;
            
        case CROSSING_LINE:
            driveWithHeading(100, cross_target_yaw, current_angle, pidStraight);
            
            if (currentMillis - lastI2C >= 10) {
                lastI2C = currentMillis;
                uint8_t raw_val = 255;
                comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
                
                if (raw_val != 255) {
                    uint8_t val = raw_val & 0x1F;  
                    if (val != 0) { 
                        state = RUNNING; 
                        lastVal = val;
                    }
                }
            }
            return;
            
case RUNNING:
            if (currentMillis - lastI2C < 10) return;
            lastI2C = currentMillis;

            uint8_t raw_val = 255; 
            comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
            if (raw_val == 255) return;
            uint8_t val = raw_val & 0x1F;  

            bool isMidLine = ((val & 0x0E) != 0) && ((val & 0x11) == 0);
            
            if (isMidLine) {
                if (!readyToStart && phase == 0) {
                    readyToStart = true;
                }
            }

            if (phase == 1 && val != 0) {
                if (val == 31) { 
                    dirMode = 2;
                    phase = 2;
                    stripeCount = 0; 
                } else if (isMidLine) { 
                    dirMode = 1;
                    phase = 2; 
                    stripeCount = 0; 
                }
            }

            if (phase == 4 && val != 0) {
                if (isMidLine || val == 31) { 
                    dirMode = 2;
                    phase = 2;
                    stripeCount = 0; 
                } else if (isMidLine) { 
                    dirMode = 1;
                    phase = 2; 
                    stripeCount = 0; 
                }
            }

            if (phase == 4 && val != 31) {
                if (isMidLine || val == 0) {
                    setMotors(0, 0);
                    state = BLINKING_END;
                    indState = IND_BLINKING_END;
                    blinkCount = 0;
                    ledState = true;
                    indPrevMillis = currentMillis;
                    for(int i = 0; i < CROSS_NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
                    pixels.show();
                    phase = 5; 
                }
            }

            if (val == 31 && lastVal != 31) { 
                if (phase == 0) {
                    if (readyToStart) {
                        phase = 1;
                        readyToStart = false; 
                    }
                }
                else if (phase == 2) {
                    if (stripeCount < 7) {
                        setMotors(0, 0); 
                        stripeCount++;
                        
                        uint32_t color = 0;
                        if (dirMode == 1) {
                            switch (stripeCount) {
                                case 1: color = pixels.Color(255, 0, 0); break;
                                case 2: color = pixels.Color(0, 255, 0); break;
                                case 3: color = pixels.Color(255, 255, 0); break;
                                case 4: color = pixels.Color(0, 0, 255); break;
                                case 5: color = pixels.Color(255, 0, 255); break;
                                case 6: color = pixels.Color(0, 255, 255); break;
                                case 7: color = pixels.Color(255, 255, 255); break;
                            }
                        } else {
                            switch (stripeCount) {
                                case 1: color = pixels.Color(255, 80, 0); break;
                                case 2: color = pixels.Color(0, 255, 128); break;
                                case 3: color = pixels.Color(0, 255, 0); break;
                                case 4: color = pixels.Color(255, 165, 0); break;
                                case 5: color = pixels.Color(255, 105, 180); break;
                                case 6: color = pixels.Color(0, 128, 255); break;
                                case 7: color = pixels.Color(128, 128, 128); break;
                            }
                        }
                        pixels.setPixelColor(stripeCount - 1, color);
                        pixels.show();
                        state = PAUSING_STRIPE;
                        prevMillis = currentMillis;
                        
                        if (stripeCount == 7) {
                            phase = 3; 
                        }
                    }
                }
                else if (phase == 3) {
                    phase = 4;
                }
            } 

            if (state == RUNNING) {
                driveWithHeading(80, cross_target_yaw, current_angle, pidStraight);
            }
            lastVal = val;
            break;
    }
}
void modeBrokenLine(bool reset) {
    static unsigned long lastI2C = 0; 
    static uint8_t lastVal = 4;

    if (reset) {
        lastI2C = 0; lastVal = 4;
        return;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return;
    uint8_t val = raw_val & 0x1F;  
    if (val == 0) val = lastVal;
    else lastVal = val;

        switch (val) {
            case 4: case 14: setMotors(90, 90); break; 
            case 12: setMotors(86, 90); break; 
            case 8:  setMotors(78, 90); break; 
            case 24: setMotors(55, 95); break; 
            case 16: setMotors(0, 125); break; 
            case 28: setMotors(0, 110); break;
            case 30: setMotors(0, 120); break; 
            case 6:  setMotors(90, 86); break; 
            case 2:  setMotors(90, 78); break; 
            case 3:  setMotors(95, 55); break; 
            case 1:  setMotors(125, 0); break; 
            case 7:  setMotors(110, 0); break; 
            case 15: setMotors(120, 0); break; 
            case 31: setMotors(80, 80); break; 
            default: setMotors(90, 90); break; 
        }
}