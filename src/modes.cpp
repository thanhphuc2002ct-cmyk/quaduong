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

// Giải mê cung bằng thuật toán ma trận kết hợp PID góc tương đối và giảm tốc độ
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
        current_target_yaw = 0.0; isInit = false;
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

    uint8_t rx_data[2] = {255, 255}; 
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data); 
    if (rx_data[0] == 255 && rx_data[1] == 255) return; 

    long currentDistance = (long)rx_data[0];     
    uint8_t val = (~rx_data[1]) & 0x1F;        
    
    if (currentState == FOLLOW_LINE && val == 0) {
        setMotors(0, 0);
        delay(1000);
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
            
            int score = 0;
            if (grid[nx][ny] == 1) score += 1000; 
            
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
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        } else {
            currentState = NODE_ARRIVED; 
            actionStartTime = currentMillis;
            if (bestDir == (currentDir + 1) % 4) { pendingTurn = TURN_RIGHT; Serial.println("-> Lua chon: RE PHAI"); }
            else if (bestDir == (currentDir + 3) % 4) { pendingTurn = TURN_LEFT; Serial.println("-> Lua chon: RE TRAI"); }
            else { pendingTurn = TURN_AROUND; Serial.println("-> Lua chon: QUAY DAU (Ngo cut)"); }
        }
        return;
    }

    if (currentState == NODE_ARRIVED) {
        driveWithHeading(100, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime >= 250) { 
            currentState = pendingTurn; 
            turnPhase = 0;
            actionStartTime = currentMillis;
            if (pendingTurn == TURN_RIGHT) current_target_yaw = normalizeAngle(current_target_yaw - 80.0);
            else if (pendingTurn == TURN_LEFT) current_target_yaw = normalizeAngle(current_target_yaw + 80.0);
            else if (pendingTurn == TURN_AROUND) current_target_yaw = normalizeAngle(current_target_yaw + 180.0);
        }
        return;
    }

    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND) {
        float error_val = calculateAngleError(current_target_yaw, current_angle);
        float error_abs = abs(error_val);
        if (error_abs < 15.0) {
            setMotors(0, 0); 
            if (turnPhase == 0) {
                turnPhase = 1;
                actionStartTime = currentMillis; 
            } else if (currentMillis - actionStartTime >= 100) { 
                if (currentState == TURN_RIGHT) currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT) currentDir = (currentDir + 3) % 4;
                else currentDir = (currentDir + 2) % 4;
                currentState = PUSH_THROUGH; 
                actionStartTime = currentMillis;
            }
        } else {
            turnPhase = 0; 
            if (currentState == TURN_RIGHT) {
                if (error_val < 0) setMotors(115, 0); 
                else setMotors(-65, 0);              
            } else if (currentState == TURN_LEFT) {
                if (error_val > 0) setMotors(0, 115); 
                else setMotors(0, -65);              
            } else {
                if (error_val > 0) setMotors(-80, 80); 
                else setMotors(80, -80);
            }
        }
        return;
    }

    if (currentState == PUSH_THROUGH) {
        driveWithHeading(100, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime > 150 && val != 0) currentState = FOLLOW_LINE;
        else if (currentMillis - actionStartTime > 600) currentState = FOLLOW_LINE;
        return;
    }

    if (currentState == REVERSE_TO_NODE) {
        driveWithHeading(-80, current_target_yaw, current_angle, pidStraight); 
        if (currentMillis - actionStartTime > 300 && val == 0) {
            setMotors(0, 0);
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

                // currentX = constrain(currentX, 0, MAX_X);
                // currentY = constrain(currentY, 0, MAX_Y);

                Serial.println("-> Lui ve nga tu truoc do de tim duong khac...");

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
            case 27: case 17: setMotors(100, 100); break; 
            case 19: setMotors(65, 85); break; 
            case 23: setMotors(40, 85); break; 
            case 7:  setMotors(20, 100); break; 
            case 15: setMotors(0, 115); break; 
            case 3:  setMotors(-20, 130); break;
            case 1:  setMotors(-40, 145); break; 
            case 25: setMotors(85, 65); break; 
            case 29: setMotors(85, 40); break; 
            case 28: setMotors(100, 20); break; 
            case 30: setMotors(115, 0); break; 
            case 24: setMotors(130, -20); break; 
            case 16: setMotors(145, -40); break; 
            case 31: setMotors(60, 60); break; 
            default: setMotors(100, 100); break; 
        }
    }
}


// Bám vạch hành trình và dừng khẩn cấp khi phát hiện chướng ngại vật phía trước
void modeObstacleAvoidance(bool reset) {
    enum ObstacleState { FOLLOW, OBSTACLE };
    static ObstacleState state = FOLLOW;
    static unsigned long lastI2C = 0;
    static uint8_t lastVal = 27; 

    if (reset) {
        state = FOLLOW; lastI2C = 0; lastVal = 27;
        return;
    }
    
    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t rx_data[2] = {255, 255};
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data);
    if (rx_data[0] == 255 && rx_data[1] == 255) return;

    long distance = (long)rx_data[0]; 
    uint8_t val = (~rx_data[1] & 0x1F); 

    if (val == 31) val = lastVal;
    else lastVal = val;

    if (distance > 0 && distance <= 10) {
        setMotors(0, 0);
        state = OBSTACLE;
    } else {
        if (state == OBSTACLE) state = FOLLOW;

        if (state == FOLLOW) {
            switch (val) {
                case 27: case 17: case 0: setMotors(70, 65); break;
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
                default: setMotors(70, 65); break;
            }
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
    static uint8_t lastVal = 27; 
    static int detectCount = 0;
    static float drop_target_yaw = 0.0;
    static int turnPhase = 0;
    static Servo gripper;
    static bool isInit = false;

    if (reset) {
        state = FOLLOW; lastI2C = 0; hasObject = false; actionTime = 0;
        lastVal = 27; detectCount = 0; drop_target_yaw = 0.0; turnPhase = 0;
        isInit = false;
        return;
    }

    if (!isInit) {
        ESP32PWM::allocateTimer(0);
        gripper.setPeriodHertz(50);
        gripper.attach(38, 500, 2400); 
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

    uint8_t rx_data[2] = {255, 255};
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data);
    if (rx_data[0] == 255 && rx_data[1] == 255) return;

    long distance = (long)rx_data[0];
    uint8_t val = (~rx_data[1] & 0x1F);

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

    if (val == 31) val = lastVal;
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

    if (val == 0) {
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
            case 27: case 17: setMotors(70, 65); break;
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
            default: setMotors(70, 65); break;
        }
    }
}
// Đếm vạch ngang giao lộ và phản hồi tín hiệu âm thanh kết hợp chớp LED đa sắc
void modeCrossroad(bool reset) {
    const int CROSS_BUZZER_PIN = 48;
    const int CROSS_LED_PIN = 38;
    const int CROSS_NUMPIXELS = 1;
    const int CROSS_START_MARK = 0; 

    enum IndState { IDLE, BEEPING_STRIPE, IND_BLINKING_END };
    enum RobotState { RUNNING, PAUSING_STRIPE, PAUSING_END, BLINKING_END, CROSSING_LINE };
    
    static IndState indState = IDLE;
    static RobotState state = RUNNING;
    static unsigned long indPrevMillis = 0;
    static int blinkCount = 0;
    static bool ledState = false;
    static int targetBeeps = 0;
    static int currentBeepCount = 0;
    static bool isBeepOn = false;
    static uint16_t stripeCount = 0;
    static uint8_t lastVal = 255;
    static bool sawMark27 = false;
    static bool enableLed = false;
    static unsigned long prevMillis = 0;
    static unsigned long lastI2C = 0;
    static Adafruit_NeoPixel pixels(CROSS_NUMPIXELS, CROSS_LED_PIN, NEO_GRB + NEO_KHZ800);
    static bool isInit = false;

    if (reset) {
        indState = IDLE; state = RUNNING; indPrevMillis = 0; blinkCount = 0;
        ledState = false; targetBeeps = 0; currentBeepCount = 0; isBeepOn = false;
        stripeCount = 0; lastVal = 255; sawMark27 = false; enableLed = false;
        prevMillis = 0; lastI2C = 0; isInit = false;
        return;
    }

    if (!isInit) {
        pinMode(CROSS_BUZZER_PIN, OUTPUT);
        digitalWrite(CROSS_BUZZER_PIN, LOW);
        pixels.begin();
        pixels.clear();
        pixels.show();
        isInit = true;
    }

    unsigned long currentMillis = millis();

    if (indState == BEEPING_STRIPE) {
        if (currentMillis - indPrevMillis >= 100) {
            indPrevMillis = currentMillis;
            if (isBeepOn) {
                digitalWrite(CROSS_BUZZER_PIN, LOW);
                isBeepOn = false;
                currentBeepCount++;
                if (currentBeepCount >= targetBeeps) indState = IDLE;
            } else {
                digitalWrite(CROSS_BUZZER_PIN, HIGH);
                isBeepOn = true;
            }
        }
    } 
    else if (indState == IND_BLINKING_END) {
        if (currentMillis - indPrevMillis >= 500) {
            indPrevMillis = currentMillis;
            ledState = !ledState; 
            if (ledState) {
                pixels.setPixelColor(0, pixels.Color(255, 255, 255));
                digitalWrite(CROSS_BUZZER_PIN, HIGH);
            } else {
                pixels.clear();
                digitalWrite(CROSS_BUZZER_PIN, LOW);
            }
            pixels.show();
            blinkCount++;
            if (blinkCount >= 20) {
                pixels.clear();
                pixels.show();
                digitalWrite(CROSS_BUZZER_PIN, LOW);
                indState = IDLE;
            }
        }
    }

    switch (state) {
        case PAUSING_STRIPE:
            if (currentMillis - prevMillis >= 3000) {
                state = CROSSING_LINE;
                setMotors(45, 40);
                prevMillis = currentMillis;
            }
            return;
        case PAUSING_END:
            if (currentMillis - prevMillis >= 10000) {
                state = CROSSING_LINE;
                setMotors(45, 40);
                prevMillis = currentMillis;
            }
            return;
        case BLINKING_END:
            if (indState != IND_BLINKING_END) {
                enableLed = false;
                state = CROSSING_LINE;
                setMotors(45, 40);
                prevMillis = currentMillis;
            }
            return;
        case CROSSING_LINE:
            if (currentMillis - prevMillis >= 400) state = RUNNING;
            return;
        case RUNNING:
            if (currentMillis - lastI2C < 10) return;
            lastI2C = currentMillis;

            uint8_t raw_val = 255; 
            comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
            if (raw_val == 255) return;
            uint8_t val = raw_val & 0x1F; 

            if (val == CROSS_START_MARK && !enableLed) enableLed = true;
            if (val == 27) sawMark27 = true;
            else if (val == 31) sawMark27 = false; 

            if (val == 0 && lastVal != 0) { 
                setMotors(0, 0);
                if (sawMark27) {
                    stripeCount = 0; 
                    if (enableLed) {
                        state = BLINKING_END;
                        indState = IND_BLINKING_END;
                        blinkCount = 0;
                        ledState = true;
                        indPrevMillis = currentMillis;
                        pixels.setPixelColor(0, pixels.Color(255, 255, 255));
                        pixels.show();
                        digitalWrite(CROSS_BUZZER_PIN, HIGH);
                    } else {
                        state = PAUSING_END;
                        prevMillis = currentMillis; 
                    }
                    sawMark27 = false;
                } 
                else {
                    if (enableLed) {
                        stripeCount++;
                        targetBeeps = stripeCount;
                        currentBeepCount = 0;
                        isBeepOn = true;
                        digitalWrite(CROSS_BUZZER_PIN, HIGH);
                        indState = BEEPING_STRIPE;
                        indPrevMillis = currentMillis;
                        
                        pixels.clear();
                        if (stripeCount == 1) pixels.setPixelColor(0, pixels.Color(255, 0, 0));
                        else if (stripeCount == 2) pixels.setPixelColor(0, pixels.Color(0, 255, 0));
                        else if (stripeCount == 3) pixels.setPixelColor(0, pixels.Color(255, 255, 0));
                        else if (stripeCount == 4) pixels.setPixelColor(0, pixels.Color(0, 0, 255));
                        else if (stripeCount == 5) pixels.setPixelColor(0, pixels.Color(128, 0, 128));
                        else pixels.setPixelColor(0, pixels.Color(0, 255, 255));
                        pixels.show();
                    }
                    state = PAUSING_STRIPE;
                    prevMillis = currentMillis; 
                }
            } 
            else if (val != 0) {
                setMotors(45, 40);
            }
            lastVal = val;
            break; 
    }
}

// Duy trì vết di chuyển bằng bộ nhớ trạng thái khi xe đi qua đoạn mất line hoặc cầu gãy
void modeBrokenLine(bool reset) {
    static unsigned long lastI2C = 0; 
    static uint8_t lastVal = 27;

    if (reset) {
        lastI2C = 0; lastVal = 27;
        return;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return;
    uint8_t val = raw_val & 0x1F; 

    if (val == 31) val = lastVal;
    else lastVal = val;

    switch (val) {
        case 27: case 17: case 0: setMotors(60, 55); break;
        case 19: setMotors(50, 60); break; 
        case 23: setMotors(35, 75); break; 
        case 7:  setMotors(0, 105); break; 
        case 15: setMotors(-30, 125); break;
        case 3:  setMotors(-50, 145); break;
        case 1:  setMotors(-70, 155); break;
        case 25: setMotors(65, 45); break; 
        case 29: setMotors(80, 25); break; 
        case 28: setMotors(110, -5); break; 
        case 30: setMotors(130, -35); break;
        case 24: setMotors(150, -55); break;
        case 16: setMotors(160, -75); break;
        default: setMotors(60, 55); break;
    }
}
