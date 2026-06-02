#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "pid.h"
#include "bsp_periph.h"
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

Master comm;
Peripheral periph;

// Trạng thái hệ thống chính
enum AppMode { MODE_IDLE, MODE_MAZE, MODE_OBSTACLE, MODE_PICK, MODE_CROSSROAD, MODE_BROKEN_LINE };
AppMode currentMode = MODE_IDLE;

// Prototype các chế độ chạy
void modeMazeSolver();
void modeObstacleAvoidance();
void modePickAndDrop();
void modeCrossroad();
void modeBrokenLine();

// Các biến toàn cục cũ của Maze solver 
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
    
    Init_MPU(MPU_SDA_PIN, MPU_SCL_PIN); 

    Serial1.begin(115200, SERIAL_8N1, 41, 42); 

    grid[0][0] = 1;
    Serial.println("HE THONG SAN SANG! Bam 1-5 tren remote IR de chon che do.");
}

void loop() {
    // 1. Quét tín hiệu IR liên tục để chuyển Mode (Non-blocking)
    uint8_t irKey = periph.Get_IR_Code();
    if (irKey != 0) {
        setMotors(0, 0); // Phanh cứng thả trôi khi đổi chế độ
        if (irKey == '1') { currentMode = MODE_MAZE; Serial.println(">> MODE 1: GIAI ME CUNG"); }
        else if (irKey == '2') { currentMode = MODE_OBSTACLE; Serial.println(">> MODE 2: DO LINE + TRANH VAT CAN"); }
        else if (irKey == '3') { currentMode = MODE_PICK; Serial.println(">> MODE 3: GAP VAT"); }
        else if (irKey == '4') { currentMode = MODE_CROSSROAD; Serial.println(">> MODE 4: QUA DUONG + DEN COI"); }
        else if (irKey == '5') { currentMode = MODE_BROKEN_LINE; Serial.println(">> MODE 5: VACH DOC / CAU GAY"); }
    }

    // 2. Chạy hàm logic tương ứng
    switch (currentMode) {
        case MODE_MAZE:         modeMazeSolver(); break;
        case MODE_OBSTACLE:     modeObstacleAvoidance(); break;
        case MODE_PICK:         modePickAndDrop(); break;
        case MODE_CROSSROAD:    modeCrossroad(); break;
        case MODE_BROKEN_LINE:  modeBrokenLine(); break;
        default:                break; // IDLE
    }
}

// Đổi tên hàm loop() giải mê cung cũ thành modeMazeSolver()
void modeMazeSolver() {
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
            case 27: case 17: setMotors(70, 70); break; 
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
            default: setMotors(70, 70); break;
        }
    }
}

// Chế độ dò line và phanh tránh vật cản (dolinechuongngai.txt)
void modeObstacleAvoidance() {
    enum ObstacleState { FOLLOW, UTURN, OBSTACLE };
    static ObstacleState state = FOLLOW;
    static unsigned long lastI2C = 0;
    static long distance = 999;
    
    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t rx_data[2] = {255, 255};
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data);
    if (rx_data[0] == 255 && rx_data[1] == 255) return;

    distance = (long)rx_data[0]; 
    uint8_t val = (~rx_data[1] & 0x1F); 

    if (distance > 0 && distance <= 10) {
        setMotors(0, 0);
        state = OBSTACLE;
    } else {
        if (state == OBSTACLE) state = FOLLOW;

        if (val == 31 && state == FOLLOW) state = UTURN;

        if (state == UTURN) {
            setMotors(100, -100);
            if (val != 31) state = FOLLOW;
        } 
        else if (state == FOLLOW) {
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

// Chế độ gắp thả vật cản bằng tay kẹp Servo (gapvat.txt)
void modePickAndDrop() {
    enum PickState { FOLLOW, UTURN, PICKING_UP, DROPPING_OFF, WAITING_CLEAR };
    static PickState state = FOLLOW;
    static unsigned long lastI2C = 0; 
    static long distance = 999;
    static bool hasObject = false;
    static unsigned long actionTime = 0;
    static int lostCount = 0;
    static uint8_t lastVal = 27; 
    static int detectCount = 0;
    static Servo gripper;
    static bool isInit = false;

    if (!isInit) {
        ESP32PWM::allocateTimer(0);
        gripper.setPeriodHertz(50);
        gripper.attach(38, 500, 2400); 
        gripper.write(0); 
        isInit = true;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t rx_data[2] = {255, 255};
    comm.I2CrequestFrom(I2C_ADDR, 2, rx_data);
    if (rx_data[0] == 255 && rx_data[1] == 255) return;

    distance = (long)rx_data[0];
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

    if (state == DROPPING_OFF) {
        setMotors(0, 0); 
        gripper.write(0);
        if (currentMillis - actionTime >= 1000) { 
            hasObject = false;
            state = WAITING_CLEAR; 
        }
        return;
    }

    if (state == WAITING_CLEAR) {
        setMotors(0, 0);
        gripper.write(0); 
        if (distance > 15 || distance == 999) state = FOLLOW;
        return;
    }

    if (state == UTURN) {
        setMotors(100, -100);
        if (val != 31) { 
            state = FOLLOW;
            lostCount = 0;
        }
        return;
    }

    if (val == 31) {
        lostCount++;
        if (lostCount >= 8) { 
            state = UTURN;
            lostCount = 0;
        } else {
            val = lastVal;
        }
    } else {
        lostCount = 0; 
        lastVal = val;
    }

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
            actionTime = currentMillis;
            state = DROPPING_OFF;
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

// Chế độ qua đường, đếm vạch, báo hiệu bằng còi và LED RGB (quaduong.txt)
void modeCrossroad() {
    // Khai báo cục bộ hoàn toàn, không dính líu đến hệ thống chung
    const int BUZZER_PIN = 48;
    const int LED_PIN = 38;
    const int NUMPIXELS = 1;
    const int START_MARK_VAL = 0; // Cập nhật lại giá trị mảng I2C của vạch bắt đầu nếu khác 0

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
    static bool isStopped = false;
    static bool enableLed = false;
    static unsigned long prevMillis = 0;
    static unsigned long lastI2C = 0;
    
    static Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
    static bool isInit = false;

    if (!isInit) {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
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
                digitalWrite(BUZZER_PIN, LOW);
                isBeepOn = false;
                currentBeepCount++;
                if (currentBeepCount >= targetBeeps) indState = IDLE;
            } else {
                digitalWrite(BUZZER_PIN, HIGH);
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
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                pixels.clear();
                digitalWrite(BUZZER_PIN, LOW);
            }
            pixels.show();
            blinkCount++;
            if (blinkCount >= 20) {
                pixels.clear();
                pixels.show();
                digitalWrite(BUZZER_PIN, LOW);
                indState = IDLE;
            }
        }
    }

    if (isStopped) return;

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
            if (currentMillis - prevMillis >= 400) {
                state = RUNNING;
            }
            return;
        case RUNNING:
            if (currentMillis - lastI2C < 10) return;
            lastI2C = currentMillis;

            uint8_t raw_val = 255; 
            comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
            if (raw_val == 255) return;
            
            uint8_t val = raw_val & 0x1F; 

            if (val == START_MARK_VAL && !enableLed) enableLed = true;
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
                        digitalWrite(BUZZER_PIN, HIGH);
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
                        digitalWrite(BUZZER_PIN, HIGH);
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

// Chế độ dò line có trí nhớ để vượt qua vạch dọc/cầu gãy (vachdoc.txt)
void modeBrokenLine() {
    static unsigned long lastI2C = 0; 
    static uint8_t lastVal = 27;

    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10) return;
    lastI2C = currentMillis;

    uint8_t raw_val = 255; 
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    if (raw_val == 255) return;
    
    uint8_t val = raw_val & 0x1F; 

    if (val == 31) {
        val = lastVal;
    } else {
        lastVal = val;
    }

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