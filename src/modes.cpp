#include "modes.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "pid.h"
#include <Preferences.h>
extern Preferences prefs;
extern Master comm;
char remoteCmd = 0;
extern float leftTrim;
extern float rightTrim;
extern unsigned long prev_time;

void modeAutoCalibrate()
{
    Serial.println("\n=== BAT DAU AUTO CALIB MOTOR ===");
    leftTrim = 1.0;
    rightTrim = 1.0;

    int attempt = 1;
    while (true)
    {
        Serial.printf("\n--- Lan calib thu %d. Cho 3s de dat xe... ---\n", attempt);
        delay(3000);

        current_angle = 0.0;
        prev_time = micros();
        unsigned long startRun = millis();

        while (millis() - startRun < 1500)
        {
            updateAngle();
            setMotors(120, 120);
            delay(10);
        }

        setMotors(0, 0);
        delay(500);
        updateAngle();

        float drift = current_angle;
        Serial.printf("Goc lech do duoc: %.2f do\n", drift);

        if (drift > 1.5)
        {
            rightTrim -= 0.05;
            if (rightTrim < 0.4) rightTrim = 0.4;
        }
        else if (drift < -1.5)
        {
            leftTrim -= 0.05;
            if (leftTrim < 0.4) leftTrim = 0.4;
        }
        else
        {
            Serial.println("-> XE DA CHAY THANG!");
            break;
        }

        if (attempt >= 20)
        {
            Serial.println("-> CANH BAO: Dat gioi han 20 lan, thoat de tranh treo may!");
            break;
        }
        attempt++;
    }

    prefs.putFloat("left", leftTrim);
    prefs.putFloat("right", rightTrim);

    Serial.println("\n=== HOAN THANH ===");
    Serial.printf("DA LUU VAO FLASH ESP32: leftTrim = %.2f | rightTrim = %.2f\n", leftTrim, rightTrim);
}

// Đọc khoảng cách từ cảm biến siêu âm HC-SR04 với timeout chống treo
long getSonarDistance()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    static long lastValidDist = 999;
    static int errorCount = 0;

    if (duration == 0)
    {
        errorCount++;
        if (errorCount >= 2)
        {
            lastValidDist = 999;
        }
        return lastValidDist;
    }

    errorCount = 0;
    long dist = duration * 0.034 / 2;
    lastValidDist = dist;
    return dist;
}

void modeMazeSolver(bool reset)
{
    static String cmdQueue = "";
    static int cmdIndex = 0;

    enum State
    {
        WAIT_INPUT,
        EXECUTE_CMD,
        FOLLOW_LINE,
        PUSH_THROUGH,
        TURN_RIGHT,
        TURN_LEFT,
        TURN_AROUND
    };
    static State currentState = WAIT_INPUT;
    static unsigned long actionStartTime = 0;
    static float current_target_yaw = 0.0;
    static int turnPhase = 0;
    static bool isInit = false;

    if (reset)
    {
        cmdQueue = "";
        cmdIndex = 0;
        currentState = WAIT_INPUT;
        actionStartTime = 0;
        current_target_yaw = 0.0;
        turnPhase = 0;
        isInit = false;
        remoteCmd = 0;
        return;
    }

    if (!isInit)
    {
        updateAngle();
        current_target_yaw = round(current_angle / 90.0) * 90.0;
        current_angle = current_target_yaw;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (remoteCmd != 0)
    {
        char cmd = remoteCmd;
        remoteCmd = 0;

        if (currentState == WAIT_INPUT)
        {
            if (cmd == 'U' || cmd == 'D' || cmd == 'L' || cmd == 'R')
            {
                cmdQueue += cmd;
                Serial.printf("-> Da them lenh: %c | Hang doi: %s\n", cmd, cmdQueue.c_str());
            }
            else if (cmd == 'O')
            {
                if (cmdQueue.length() > 0)
                {
                    Serial.printf("=> DA XAC NHAN HANG DOI LENH: %s. Bam '#' de chay!\n", cmdQueue.c_str());
                }
            }
            else if (cmd == '#')
            {
                if (cmdQueue.length() > 0)
                {
                    Serial.println("=> BAT DAU CHAY HANG DOI LENH!");
                    cmdIndex = 0;
                    currentState = EXECUTE_CMD;
                }
            }
        }
    }

    if (currentState == WAIT_INPUT)
    {
        setMotors(0, 0);
        return;
    }

    static unsigned long lastI2CPoll = 0;
    if (currentMillis - lastI2CPoll < 10) return;
    lastI2CPoll = currentMillis;

    uint8_t raw_val = 0;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    uint8_t val = raw_val & 0x0F;

    switch (currentState)
    {
    case EXECUTE_CMD:
        if (cmdIndex < cmdQueue.length())
        {
            char c = cmdQueue[cmdIndex++];
            Serial.printf("=> Dang thuc thi lenh: %c\n", c);
            if (c == 'U')
            {
                currentState = FOLLOW_LINE;
            }
            else if (c == 'R')
            {
                current_target_yaw = normalizeAngle(current_target_yaw - 90.0);
                currentState = TURN_RIGHT;
                turnPhase = 0;
            }
            else if (c == 'L')
            {
                current_target_yaw = normalizeAngle(current_target_yaw + 90.0);
                currentState = TURN_LEFT;
                turnPhase = 0;
            }
            else if (c == 'D')
            {
                current_target_yaw = normalizeAngle(current_target_yaw + 180.0);
                currentState = TURN_AROUND;
                turnPhase = 0;
            }
        }
        else
        {
            Serial.println("=> DA HOAN THANH TOAN BO LENH!");
            cmdQueue = "";
            currentState = WAIT_INPUT;
        }
        break;

    case FOLLOW_LINE:
        // Đếm số bit 1. Nếu >= 3 nghĩa là có từ 3 mắt trở lên chạm line (chống nhiễu mất 1 mắt)
        if (__builtin_popcount(val) >= 3)
        {
            currentState = PUSH_THROUGH;
            actionStartTime = millis();
        }
        else
        {
            switch (val)
            {
            case 6: setMotors(120, 120); break;
            case 4: setMotors(120, 116); break;
            case 12: setMotors(125, 85); break;
            case 8: setMotors(140, 30); break;
            case 2: setMotors(116, 120); break;
            case 3: setMotors(85, 125); break;
            case 1: setMotors(30, 140); break;
            case 0: driveWithHeading(110, current_target_yaw, current_angle, pidStraight); break;
            default: driveWithHeading(120, current_target_yaw, current_angle, pidStraight); break;
            }
        }
        break;

    case PUSH_THROUGH:
        driveWithHeading(75, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime >= 500)
        {
            setMotors(0, 0);
            delay(50);  
            currentState = EXECUTE_CMD;
        }
        break;

    case TURN_RIGHT:
    case TURN_LEFT:
    case TURN_AROUND:
    {
        float error_val = calculateAngleError(current_target_yaw, current_angle);
        float error_abs = abs(error_val);
        bool caughtLine = (error_abs < 40.0) && (val == 6 || val == 4 || val == 2 || val == 12 || val == 3);

        if (error_abs < 3.0 || caughtLine || turnPhase == 1)
        {
            setMotors(0, 0);
            if (turnPhase == 0)
            {
                turnPhase = 1;
                actionStartTime = currentMillis;
            }
            else if (currentMillis - actionStartTime >= 100)
            {
                currentState = FOLLOW_LINE;
            }
        }
        else
        {
            turnPhase = 0;
            driveWithHeading(0, current_target_yaw, current_angle, pidTurn);
        }
        break;
    }
    }
}

// Giải mê cung bằng thuật toán ma trận kết hợp PID góc và PWM thuần túy
void modeMazeSolver_OLD(bool reset)
{
    const int MAX_X = 2;
    const int MAX_Y = 1;
    static int grid[3][2] = {0};
    static int currentX = 0;
    static int currentY = 0;
    static int currentDir = 0;

    enum State
    {
        FOLLOW_LINE,
        NODE_ARRIVED,
        TURN_RIGHT,
        TURN_LEFT,
        TURN_AROUND,
        PUSH_THROUGH,
        REVERSE_TO_NODE,
        FINISHED
    };
    static State currentState = FOLLOW_LINE;
    static State pendingTurn = TURN_RIGHT;
    static unsigned long actionStartTime = 0;
    static int turnPhase = 0;
    static int obstacleCount = 0;
    static float current_target_yaw = 0.0;
    static bool isInit = false;

    if (reset)
    {
        memset(grid, 0, sizeof(grid));
        currentX = 0;
        currentY = 0;
        currentDir = 0;
        currentState = FOLLOW_LINE;
        pendingTurn = TURN_RIGHT;
        actionStartTime = 0;
        turnPhase = 0;
        obstacleCount = 0;
        current_target_yaw = 0.0;
        current_angle = 0.0;
        isInit = false;
        return;
    }

    if (!isInit)
    {
        grid[0][0] = 1;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (currentState == FINISHED)
    {
        setMotors(0, 0);
        return;
    }

    if (currentMillis - actionStartTime < 10 && currentState != NODE_ARRIVED && currentState != TURN_RIGHT && currentState != TURN_LEFT && currentState != TURN_AROUND && currentState != PUSH_THROUGH && currentState != REVERSE_TO_NODE)
    {
        static unsigned long lastI2CPoll = 0;
        if (currentMillis - lastI2CPoll < 10)
            return;
        lastI2CPoll = currentMillis;
    }

    uint8_t raw_val = 0;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);

    long currentDistance = getSonarDistance();
    uint8_t val = raw_val & 0x0F;

    if (currentState == FOLLOW_LINE && (val == 15 || val == 14 || val == 13 || val == 11 || val == 7))
    {
        setMotors(0, 0);
        delay(500);
        updateAngle();
        currentMillis = millis();

        if (currentDir == 0)
            currentY++;
        else if (currentDir == 1)
            currentX++;
        else if (currentDir == 2)
            currentY--;
        else if (currentDir == 3)
            currentX--;

        currentX = constrain(currentX, 0, MAX_X);
        currentY = constrain(currentY, 0, MAX_Y);

        grid[currentX][currentY] = 1;

        Serial.printf("\n--- DEN NGA TU: X=%d, Y=%d (Huong xe hien tai: %d) ---\n", currentX, currentY, currentDir);

        if (currentX == MAX_X && currentY == MAX_Y)
        {
            currentState = FINISHED;
            Serial.printf(">> HOAN THANH! Da toi dich (%d,%d)\n", MAX_X, MAX_Y);
            return;
        }

        int bestDir = -1;
        int bestScore = 9999;
        bool hasUnvisited = false;

        auto evaluateDirection = [&](int dir, int nx, int ny)
        {
            if (nx < 0 || nx > MAX_X || ny < 0 || ny > MAX_Y)
                return;
            if (grid[nx][ny] == 2)
                return;
            if (grid[nx][ny] == 0)
                hasUnvisited = true;

            int freeWays = 0;
            if (ny + 1 <= MAX_Y && grid[nx][ny + 1] != 2)
                freeWays++;
            if (nx + 1 <= MAX_X && grid[nx + 1][ny] != 2)
                freeWays++;
            if (ny - 1 >= 0 && grid[nx][ny - 1] != 2)
                freeWays++;
            if (nx - 1 >= 0 && grid[nx - 1][ny] != 2)
                freeWays++;

            if (freeWays <= 1 && !(nx == MAX_X && ny == MAX_Y))
            {
                grid[nx][ny] = 2;
                return;
            }

            int score = 0;
            if (grid[nx][ny] == 1)
                score += 50;

            score += (abs(MAX_X - nx) + abs(MAX_Y - ny)) * 10;

            int relDir = (dir - currentDir + 4) % 4;
            if (relDir == 0)
                score += 0;
            else if (relDir == 1)
                score += 20;
            else if (relDir == 3)
                score += 20;
            else if (relDir == 2)
                score += 200;

            if (score < bestScore)
            {
                bestScore = score;
                bestDir = dir;
            }
        };

        evaluateDirection(0, currentX, currentY + 1);
        evaluateDirection(1, currentX + 1, currentY);
        evaluateDirection(2, currentX, currentY - 1);
        evaluateDirection(3, currentX - 1, currentY);

        if (bestDir == -1)
        {
            Serial.println("-> MAZE VO NGHIEM! Tat ca cac huong deu bi chan.");
            currentState = FINISHED;
            return;
        }

        if (bestDir == currentDir)
        {
            Serial.println("-> Lua chon: DI THANG");
            current_target_yaw = round(current_angle / 90.0) * 90.0;
            current_angle = current_target_yaw;
            currentState = PUSH_THROUGH;
            actionStartTime = currentMillis;
        }
        else
        {
            currentState = NODE_ARRIVED;
            actionStartTime = currentMillis;
            current_target_yaw = round(current_angle / 90.0) * 90.0;
            current_angle = current_target_yaw;
            if (bestDir == (currentDir + 1) % 4)
            {
                pendingTurn = TURN_RIGHT;
                Serial.println("-> Lua chon: RE PHAI");
            }
            else if (bestDir == (currentDir + 3) % 4)
            {
                pendingTurn = TURN_LEFT;
                Serial.println("-> Lua chon: RE TRAI");
            }
            else
            {
                pendingTurn = TURN_AROUND;
                Serial.println("-> Lua chon: QUAY DAU (Ngo cut)");
            }
        }
        return;
    }

    if (currentState == NODE_ARRIVED)
    {
        int pushSpeed = 50 + (currentMillis - actionStartTime) / 4;
        if (pushSpeed > 65)
            pushSpeed = 65;
        driveWithHeading(pushSpeed, current_target_yaw, current_angle, pidStraight);

        if (currentMillis - actionStartTime >= 250)
        {
            setMotors(0, 0);
            delay(50);

            currentState = pendingTurn;

            turnPhase = 0;
            actionStartTime = millis(); // Cập nhật lại mốc thời gian sau delay

            if (pendingTurn == TURN_RIGHT)
                current_target_yaw = normalizeAngle(current_target_yaw - 90);
            else if (pendingTurn == TURN_LEFT)
                current_target_yaw = normalizeAngle(current_target_yaw + 90);
            else if (pendingTurn == TURN_AROUND)
                current_target_yaw = normalizeAngle(current_target_yaw + 180.0);
        }
        return;
    }

    if (currentState == TURN_RIGHT || currentState == TURN_LEFT || currentState == TURN_AROUND)
    {
        float error_val = calculateAngleError(current_target_yaw, current_angle);
        float error_abs = abs(error_val);

        bool caughtLine = (error_abs < 50.0) && (val == 6 || val == 4 || val == 2 || val == 12 || val == 3);

        if (error_abs < 3.0 || caughtLine || turnPhase == 1)
        {
            setMotors(0, 0);
            if (turnPhase == 0)
            {
                turnPhase = 1; // Khóa trạng thái phanh
                actionStartTime = currentMillis;
            }
            else if (currentMillis - actionStartTime >= 100)
            {
                if (currentState == TURN_RIGHT)
                    currentDir = (currentDir + 1) % 4;
                else if (currentState == TURN_LEFT)
                    currentDir = (currentDir + 3) % 4;
                else
                    currentDir = (currentDir + 2) % 4;

                currentState = FOLLOW_LINE;
                actionStartTime = currentMillis;
            }
        }
        else
        {
            turnPhase = 0;
            driveWithHeading(0, current_target_yaw, current_angle, pidTurn);
        }
        return;
    }

    if (currentState == PUSH_THROUGH)
    {
        int pushSpeed = 55 + (currentMillis - actionStartTime) / 3;
        if (pushSpeed > 75)
            pushSpeed = 75;
        driveWithHeading(pushSpeed, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime > 250 && val != 15 && val != 14 && val != 13 && val != 11 && val != 7)
        {
            currentState = FOLLOW_LINE;
        }
        else if (currentMillis - actionStartTime > 800)
        { // Timeout an toàn 800ms
            currentState = FOLLOW_LINE;
        }
        return;
    }
    if (currentState == REVERSE_TO_NODE)
    {
        driveWithHeading(-70, current_target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime > 300 && (val == 15 || val == 14 || val == 13 || val == 11 || val == 7))
        {
            setMotors(0, 0);
            current_angle = current_target_yaw;
            delay(50);
            currentState = FOLLOW_LINE;
        }
        return;
    }

    if (currentState == FOLLOW_LINE)
    {
        if (currentDistance > 0 && currentDistance <= 5)
        {
            obstacleCount++;
            if (obstacleCount >= 3)
            {
                setMotors(0, 0);
                delay(1000);
                updateAngle();
                int nx = currentX, ny = currentY;
                if (currentDir == 0)
                    ny++;
                else if (currentDir == 1)
                    nx++;
                else if (currentDir == 2)
                    ny--;
                else if (currentDir == 3)
                    nx--;

                if (nx >= 0 && nx <= MAX_X && ny >= 0 && ny <= MAX_Y)
                {
                    grid[nx][ny] = 2;
                    Serial.printf("!!! PHAT HIEN VAT CAN TAI: X=%d, Y=%d !!! Da cap nhat ban do.\n", nx, ny);
                }

                if (currentDir == 0)
                    currentY--;
                else if (currentDir == 1)
                    currentX--;
                else if (currentDir == 2)
                    currentY++;
                else if (currentDir == 3)
                    currentX++;
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
        }
        else
        {
            obstacleCount = 0;
        }

        switch (val)
        {
        case 6:
            setMotors(120, 120);
            break;
        case 4:
            setMotors(120, 116);
            break;
        case 12:
            setMotors(125, 85);
            break;
        case 8:
            setMotors(140, 30);
            break;
        case 2:
            setMotors(116, 120);
            break;
        case 3:
            setMotors(85, 125);
            break;
        case 1:
            setMotors(30, 140);
            break;
        case 15:
            driveWithHeading(110, current_target_yaw, current_angle, pidStraight);
            break;
        case 0:
            driveWithHeading(110, current_target_yaw, current_angle, pidStraight);
            break;
        default:
            driveWithHeading(120, current_target_yaw, current_angle, pidStraight);
            break;
        }
    }
}

// Điều khiển xe bằng Remote thông qua MPU6050
void modeRemoteControl(bool reset)
{
    enum RemoteState
    {
        IDLE,
        MOVING_FORWARD,
        MOVING_BACKWARD,
        TURNING
    };
    static RemoteState state = IDLE;
    static unsigned long actionStartTime = 0;
    static float target_yaw = 0.0;
    static bool isInit = false;

    if (reset)
    {
        state = IDLE;
        actionStartTime = 0;
        target_yaw = 0.0;
        isInit = false;
        remoteCmd = 0;
        return;
    }

    if (!isInit)
    {
        updateAngle();
        target_yaw = current_angle;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    switch (state)
    {
    case IDLE:
        setMotors(0, 0);
        if (remoteCmd == 'U')
        {
            state = MOVING_FORWARD;
            target_yaw = current_angle;
            actionStartTime = currentMillis;
            remoteCmd = 0;
        }
        else if (remoteCmd == 'D')
        {
            state = MOVING_BACKWARD;
            target_yaw = current_angle;
            actionStartTime = currentMillis;
            remoteCmd = 0;
        }
        else if (remoteCmd == 'L')
        {
            state = TURNING;
            target_yaw = normalizeAngle(current_angle + 90.0);
            remoteCmd = 0;
        }
        else if (remoteCmd == 'R')
        {
            state = TURNING;
            target_yaw = normalizeAngle(current_angle - 90.0);
            remoteCmd = 0;
        }
        else
        {
            remoteCmd = 0;
        }
        break;

    case MOVING_FORWARD:
        driveWithHeading(120, target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime >= 2000)
        {
            state = IDLE;
        }
        break;

    case MOVING_BACKWARD:
        driveWithHeading(-120, target_yaw, current_angle, pidStraight);
        if (currentMillis - actionStartTime >= 2000)
        {
            state = IDLE;
        }
        break;

    case TURNING:
    {
        float error_val = calculateAngleError(target_yaw, current_angle);
        if (abs(error_val) < 3.0)
        {
            setMotors(0, 0);
            state = IDLE;
        }
        else
        {
            driveWithHeading(0, target_yaw, current_angle, pidTurn);
        }
        break;
    }
    }
}

void modeObstacleAvoidance(bool reset)
{
    enum ObstacleState
    {
        FOLLOW,
        TURN_AROUND_FIND_ZERO,
        TURN_AROUND_FIND_LINE,
        TURN_BACK_180
    };
    static ObstacleState state = FOLLOW;
    static unsigned long lastI2C = 0;
    static uint8_t lastVal = 6;
    static float initial_yaw = 0.0;
    static bool isInit = false;
    extern long current_distance;

    if (reset)
    {
        state = FOLLOW;
        lastI2C = 0;
        lastVal = 6;
        initial_yaw = 0.0;
        isInit = false;
        return;
    }

    if (!isInit)
    {
        updateAngle();
        isInit = true;
    }

    updateAngle(); // Liên tục cập nhật MPU để đo xem xe đã xoay bao nhiêu độ
    unsigned long currentMillis = millis();

    if (state == FOLLOW && current_distance > 0 && current_distance <= 7)
    {
        setMotors(0, 0);
        delay(200);
        initial_yaw = current_angle; // Lưu lại góc ban đầu ngay khi gặp vật cản
        state = TURN_AROUND_FIND_ZERO;
        return;
    }

    // Bỏ qua thời gian đợi I2C nếu đang ở pha bẻ góc bằng PID
    if (state != TURN_BACK_180 && currentMillis - lastI2C < 10)
        return;
    lastI2C = currentMillis;

    uint8_t raw_val = 0;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    uint8_t val = raw_val & 0x0F;

    if (state == TURN_AROUND_FIND_ZERO)
    {
        setMotors(-115, 115);
        if (val == 0)
        {
            state = TURN_AROUND_FIND_LINE;
        }
        // Đề phòng lỗi xoay vòng vô tận ngay từ pha tìm nền trắng
        if (abs(current_angle - initial_yaw) > 720.0)
        {
            setMotors(0, 0);
            delay(100);
            initial_yaw = normalizeAngle(initial_yaw + 180.0);
            state = TURN_BACK_180;
        }
        return;
    }
    else if (state == TURN_AROUND_FIND_LINE)
    {
        setMotors(-115, 115);
        if (val == 6 || val == 4 || val == 2)
        {
            setMotors(0, 0);
            delay(100);
            lastVal = val;
            state = FOLLOW;
        }
        // NẾU QUAY QUÁ 2 VÒNG (720 ĐỘ) MÀ VẪN KHÔNG THẤY LINE
        else if (abs(current_angle - initial_yaw) > 720.0) 
        {
            setMotors(0, 0);
            delay(100);
            // Thiết lập mục tiêu là góc ngược lại 180 độ so với lúc gặp vật cản
            initial_yaw = normalizeAngle(initial_yaw + 180.0); 
            state = TURN_BACK_180;
        }
        return;
    }
    else if (state == TURN_BACK_180)
    {
        float error_val = calculateAngleError(initial_yaw, current_angle);
        if (abs(error_val) < 3.0)
        {
            setMotors(0, 0);
            delay(100);
            state = FOLLOW; // Quay lưng đúng 180 độ xong thì tiếp tục trạng thái bám line
        }
        else
        {
            driveWithHeading(0, initial_yaw, current_angle, pidTurn); // Sử dụng PID để ép góc xoay mượt mà
        }
        return;
    }

    if (val == 0)
        val = lastVal;
    else
        lastVal = val;

    if (state == FOLLOW)
    {
        switch (val)
        {
        case 6:
            setMotors(130, 130);
            break;
        case 4:
            setMotors(130, 126);
            break;
        case 12:
            setMotors(135, 95);
            break;
        case 8:
            setMotors(150, 40);
            break;
        case 2:
            setMotors(126, 130);
            break;
        case 3:
            setMotors(95, 135);
            break;
        case 1:
            setMotors(40, 150);
            break;
        case 15:
            setMotors(120, 120);
            break;
        default:
            setMotors(130, 130);
            break;
        }
    }
}
// Điều khiển cơ cấu servo kẹp nhả vật thể dựa trên tín hiệu siêu âm
void modePickAndDrop(bool reset)
{
    enum PickState
    {
        FOLLOW,
        PICKING_UP,
        TURN_RIGHT_DROP,
        DROPPING_ACTION,
        TURN_LEFT_BACK
    };
    static PickState state = FOLLOW;
    static unsigned long lastI2C = 0;
    static bool hasObject = false;
    static unsigned long actionTime = 0;
    static uint8_t lastVal = 6;
    static int detectCount = 0;
    static float drop_target_yaw = 0.0;
    static int turnPhase = 0;
    static Servo gripper;
    static bool isInit = false;

    if (reset)
    {
        state = FOLLOW;
        lastI2C = 0;
        hasObject = false;
        actionTime = 0;
        lastVal = 6;
        detectCount = 0;
        drop_target_yaw = 0.0;
        turnPhase = 0;
        isInit = false;
        return;
    }

    if (!isInit)
    {
        ESP32PWM::allocateTimer(0);
        gripper.setPeriodHertz(50);
        gripper.attach(48, 500, 2400);
        gripper.write(0);
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (state == TURN_RIGHT_DROP || state == TURN_LEFT_BACK)
    {
        float error_val = calculateAngleError(drop_target_yaw, current_angle);
        if (abs(error_val) < 10.0)
        {
            setMotors(0, 0);
            if (turnPhase == 0)
            {
                turnPhase = 1;
                actionTime = currentMillis;
            }
            else if (currentMillis - actionTime >= 100)
            {
                if (state == TURN_RIGHT_DROP)
                {
                    state = DROPPING_ACTION;
                    actionTime = currentMillis;
                }
                else
                {
                    state = FOLLOW;
                }
            }
        }
        else
        {
            turnPhase = 0;
            if (state == TURN_RIGHT_DROP)
            {
                if (error_val < 0)
                    setMotors(80, 0);
                else
                    setMotors(-50, 0);
            }
            else
            {
                if (error_val > 0)
                    setMotors(0, 80);
                else
                    setMotors(0, -50);
            }
        }
        return;
    }

    if (currentMillis - lastI2C < 10)
        return;
    lastI2C = currentMillis;

    uint8_t raw_val = 0;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);

    long distance = getSonarDistance();
    uint8_t val = raw_val & 0x0F;

    if (state == PICKING_UP)
    {
        setMotors(0, 0);
        gripper.write(90);
        if (currentMillis - actionTime >= 3000)
        {
            hasObject = true;
            state = FOLLOW;
        }
        return;
    }

    if (state == DROPPING_ACTION)
    {
        setMotors(0, 0);
        gripper.write(0);
        if (currentMillis - actionTime >= 1000)
        {
            hasObject = false;
            drop_target_yaw = normalizeAngle(current_angle + 90.0);
            state = TURN_LEFT_BACK;
            turnPhase = 0;
        }
        return;
    }

    if (val == 0)
        val = lastVal;
    else
        lastVal = val;

    if (!hasObject && distance > 0 && distance <= 10)
    {
        detectCount++;
        if (detectCount >= 3)
        {
            actionTime = currentMillis;
            state = PICKING_UP;
            detectCount = 0;
            return;
        }
    }
    else
    {
        detectCount = 0;
    }

    if (val == 15)
    {
        if (hasObject)
        {
            drop_target_yaw = normalizeAngle(current_angle - 90.0);
            state = TURN_RIGHT_DROP;
            turnPhase = 0;
            return;
        }
        else
        {
            setMotors(70, 65);
        }
    }
    else
    {
        switch (val)
        {
        case 6:
            setMotors(120, 120);
            break;
        case 4:
            setMotors(120, 115);
            break;
        case 12:
            setMotors(120, 95);
            break;
        case 8:
            setMotors(155, 0);
            break;
        case 2:
            setMotors(115, 120);
            break;
        case 3:
            setMotors(95, 120);
            break;
        case 1:
            setMotors(0, 155);
            break;
        case 15:
            setMotors(120, 120);
            break;
        default:
            setMotors(120, 120);
            break;
        }
    }
}
// Đếm vạch ngang giao lộ và phản hồi tín hiệu âm thanh kết hợp chớp LED đa sắc
const int CROSS_LED_PIN = 38;
const int CROSS_NUMPIXELS = 8;
Adafruit_NeoPixel pixels(CROSS_NUMPIXELS, CROSS_LED_PIN, NEO_GRB + NEO_KHZ800);

void initCrossroadLED()
{
    pixels.begin();
    pixels.setBrightness(50);
    pixels.clear();
    pixels.show();
}

// Hàm hiển thị đèn LED tương ứng với từng Mode (Mode 1 -> LED 1, Mode 2 -> LED 2,...)
void displayModeLED(int modeNum)
{
    pixels.clear();
    if (modeNum >= 1 && modeNum <= 6)
    {
        pixels.setPixelColor(modeNum - 1, pixels.Color(0, 0, 255)); 
    }
    pixels.show();
}

void showButtonHoldProgress(unsigned long heldTime)
{
    if (heldTime < 500)
    {
        pixels.clear();
    }
    else if (heldTime >= 500 && heldTime < 1000)
    {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    }
    else if (heldTime >= 1000 && heldTime < 1500)
    {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.setPixelColor(1, pixels.Color(255, 255, 0));
    }
    else if (heldTime >= 1500 && heldTime < 5000)
    {
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(255, 0, 0));
        pixels.setPixelColor(1, pixels.Color(255, 255, 0));
        pixels.setPixelColor(2, pixels.Color(0, 255, 0));
    }
    else if (heldTime >= 5000)
    {
        if ((millis() / 150) % 2 == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                pixels.setPixelColor(i, pixels.Color(255, 0, 0));
            }
        }
        else
        {
            pixels.clear();
        }
    }
    pixels.show();
}

void clearLEDs()
{
    pixels.clear();
    pixels.show();
}

void setAllLEDs(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < CROSS_NUMPIXELS; i++)
    {
        pixels.setPixelColor(i, pixels.Color(r, g, b));
    }
    pixels.show();
}

void triggerModeChangeSequence()
{
    for (int i = 0; i < 8; i++)
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    delay(150);
    pixels.clear();
    pixels.show();
}

void triggerStartSequence()
{
    setMotors(0, 0);
    for (int i = 0; i < 8; i++)
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    delay(150);
    pixels.clear();
    pixels.show();
    delay(150);
    for (int i = 0; i < 8; i++)
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    delay(150);
    pixels.clear();
    pixels.show();
    delay(150);
    for (int i = 0; i < 8; i++)
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    pixels.show();
    delay(250);
    pixels.clear();
    pixels.show();
    delay(150);

    extern unsigned long prev_time;
    prev_time = micros(); // Xóa tích lũy thời gian ảo do delay gây ra
}

void modeCrossroad(bool reset)
{
    enum IndState
    {
        IDLE,
        IND_BLINKING_END
    };
    enum RobotState
    {
        RUNNING,
        PAUSING_STRIPE,
        PAUSING_END,
        BLINKING_END,
        CROSSING_LINE
    };

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
    static bool isInit = false;
    static unsigned long startRunTime = 0; 
    static int zeroCount = 0;
    static bool readyToCountStripe = false;

    if (reset)
    {
        indState = IDLE;
        state = RUNNING;
        indPrevMillis = 0;
        blinkCount = 0;
        ledState = false;
        stripeCount = 0;
        lastVal = 15;
        prevMillis = 0;
        lastI2C = 0;
        phase = 0;
        dirMode = 0;
        readyToStart = false;
        isInit = false;
        startRunTime = millis(); 
        zeroCount = 0;
        readyToCountStripe = false;

        updateAngle();
        cross_target_yaw = current_angle;

        pixels.clear();
        pixels.show();
        return;
    }

    // LẤY GÓC HIỆN TẠI LÀM TIÊU CHUẨN NGAY KHI VÀO MODE
    if (!isInit)
    {
        updateAngle();
        cross_target_yaw = current_angle;
        isInit = true;
    }

    updateAngle();
    unsigned long currentMillis = millis();

    if (indState == IND_BLINKING_END)
    {
        const uint32_t rainbowColors[7] = {
            pixels.Color(255, 0, 0), pixels.Color(0, 255, 0), pixels.Color(255, 255, 0),
            pixels.Color(0, 0, 255), pixels.Color(255, 0, 255), pixels.Color(0, 255, 255), pixels.Color(255, 255, 255)
        };
        const uint32_t orangeColors[7] = {
            pixels.Color(255, 80, 0), pixels.Color(0, 255, 128), pixels.Color(0, 255, 0),
            pixels.Color(255, 165, 0), pixels.Color(255, 105, 180), pixels.Color(0, 128, 255), pixels.Color(128, 128, 128)
        };
        const uint32_t* currentColors = (dirMode == 2) ? rainbowColors : orangeColors;

        // LED 8 (index 7) liên tục thay đổi màu mỗi 150ms mà không bị chớp tắt
        static unsigned long led8Millis = 0;
        static int led8Index = 0;
        if (currentMillis - led8Millis >= 150)
        {
            led8Millis = currentMillis;
            led8Index = (led8Index + 1) % 7;
            pixels.setPixelColor(7, currentColors[led8Index]);
            pixels.show();
        }

        // 7 LED đầu chớp nháy màu tương ứng mỗi 500ms
        if (currentMillis - indPrevMillis >= 500)
        {
            indPrevMillis = currentMillis;
            ledState = !ledState;
            
            if (ledState)
            {
                for (int i = 0; i < 7; i++)
                    pixels.setPixelColor(i, currentColors[i]);
            }
            else
            {
                for (int i = 0; i < 7; i++)
                    pixels.setPixelColor(i, 0);
            }
            pixels.show();
            
            blinkCount++;
            if (blinkCount >= 10)
            {
                pixels.clear();
                pixels.show();
                indState = IDLE;
            }
        }
    }

    switch (state)
    {
    case PAUSING_STRIPE:
        if (currentMillis - prevMillis >= 2000)
        {
            state = CROSSING_LINE;
            startRunTime = currentMillis; // Bắt đầu tăng tốc mượt từ đây
        }
        return;

    case PAUSING_END:
        if (currentMillis - prevMillis >= 10000)
        {
            state = CROSSING_LINE;
            prevMillis = currentMillis;
            startRunTime = currentMillis;
        }
        return;

    case BLINKING_END:
        setMotors(0, 0);
        if (indState != IND_BLINKING_END)
        {
            extern bool isRunning;
            isRunning = false;

            Serial.println("-> DA HOAN THANH QUA DUONG! Dang cho lenh moi...");
        }
        return;

    case CROSSING_LINE:
    {
        int push_speed = 80;
        unsigned long elapsed = currentMillis - startRunTime;
        if (elapsed < 400)
        {
            push_speed = map(elapsed, 0, 400, 40, 80);
        }
        driveWithHeading(push_speed, cross_target_yaw, current_angle, pidStraight);

        if (currentMillis - lastI2C >= 10)
        {
            lastI2C = currentMillis;
            uint8_t raw_val = 0;
            comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);

            uint8_t val = raw_val & 0x0F;
            
            if (val == 0)
            {
                state = RUNNING;
                lastVal = val;
                // KHÔNG reset startRunTime ở đây, để xe giữ nguyên gia tốc tiếp nối sang RUNNING
            }
        }
        return;
    }

    case RUNNING:
        if (currentMillis - lastI2C < 10)
            return;
        lastI2C = currentMillis;

        uint8_t raw_val = 0;
        comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
        uint8_t val = raw_val & 0x0F;

        if (val == 0)
        {
            zeroCount++;
            if (zeroCount >= 5)
            {
                readyToCountStripe = true;
            }
        }
        else
        {
            zeroCount = 0;
        }

        // Kiểm tra 1 hoặc 2 mắt giữa đang có vạch (giá trị 2, 4, 6)
        bool isMidLine = ((val & 0x06) != 0) && ((val & 0x09) == 0);

        if (phase == 0)
        { // ĐỢI VẠCH XUẤT PHÁT
            if (isMidLine)
            {
                readyToStart = true;
            }
            // Nếu đã bắt được vạch giữa trước đó, và giờ đạp lên full line (từ 3 mắt trở lên)
            if (readyToStart && __builtin_popcount(val) >= 3 && __builtin_popcount(lastVal) < 3)
            {
                phase = 1; // Bắt đầu vào vạch xuất phát, nhưng KHÔNG tăng stripeCount
            }
        }
        else if (phase == 1)
        { // RỜI VẠCH XUẤT PHÁT ĐỂ XÁC ĐỊNH CHIỀU
            if (__builtin_popcount(val) < 3)
            { // Xe vừa trượt khỏi vạch full line đầu tiên
                if (val == 0)
                {
                    dirMode = 1; // Chiều 1: Mid -> Full -> Trắng (Màu cam xuất phát)
                    phase = 2;
                    stripeCount = 0;
                }
                else if (isMidLine)
                {
                    dirMode = 2; // Chiều 2: Mid -> Full -> Mid (Màu đỏ xuất phát)
                    phase = 2;
                    stripeCount = 0;
                }
            }
        }
        else if (phase == 2)
        { // ĐẾM 7 VẠCH
            if (__builtin_popcount(val) >= 3 && readyToCountStripe)
            {
                readyToCountStripe = false;
                zeroCount = 0;
                setMotors(0, 0);
                stripeCount++;

                uint32_t color = 0;
                if (dirMode == 2)
                { // Chiều cầu vồng
                    switch (stripeCount)
                    {
                    case 1:
                        color = pixels.Color(255, 0, 0);
                        break;
                    case 2:
                        color = pixels.Color(0, 255, 0);
                        break;
                    case 3:
                        color = pixels.Color(255, 255, 0);
                        break;
                    case 4:
                        color = pixels.Color(0, 0, 255);
                        break;
                    case 5:
                        color = pixels.Color(255, 0, 255);
                        break;
                    case 6:
                        color = pixels.Color(0, 255, 255);
                        break;
                    case 7:
                        color = pixels.Color(255, 255, 255);
                        break;
                    }
                }
                else
                { // Chiều màu Cam
                    switch (stripeCount)
                    {
                    case 1:
                        color = pixels.Color(255, 80, 0);
                        break;
                    case 2:
                        color = pixels.Color(0, 255, 128);
                        break;
                    case 3:
                        color = pixels.Color(0, 255, 0);
                        break;
                    case 4:
                        color = pixels.Color(255, 165, 0);
                        break;
                    case 5:
                        color = pixels.Color(255, 105, 180);
                        break;
                    case 6:
                        color = pixels.Color(0, 128, 255);
                        break;
                    case 7:
                        color = pixels.Color(128, 128, 128);
                        break;
                    }
                }
                pixels.setPixelColor(stripeCount - 1, color);
                pixels.show();
                state = PAUSING_STRIPE;
                prevMillis = currentMillis;

                if (stripeCount == 7)
                {
                    phase = 3; // Chuyển sang chờ vạch đích
                }
            }
        }
        else if (phase == 3)
        { // CHỜ VẠCH ĐÍCH (VẠCH THỨ 8)
            if (__builtin_popcount(val) >= 3 && readyToCountStripe)
            {
                readyToCountStripe = false;
                zeroCount = 0;
                setMotors(0, 0); // Phanh khẩn cấp đúng trên vạch đích
                state = BLINKING_END;
                indState = IND_BLINKING_END;
                blinkCount = 0;
                ledState = true;
                indPrevMillis = currentMillis;
                for (int i = 0; i < CROSS_NUMPIXELS; i++)
                    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
                pixels.show();
                phase = 4; // Khóa chết bộ đếm
            }
        }

        if (state == RUNNING)
        {
            int current_speed = 80;
            unsigned long elapsed = currentMillis - startRunTime;

            if (elapsed < 500)
            {
                current_speed = map(elapsed, 0, 500, 0, 80);
            }

            driveWithHeading(current_speed, cross_target_yaw, current_angle, pidStraight);
        }
        lastVal = val;
        break;
    }
}
void modeBrokenLine(bool reset)
{
    static unsigned long lastI2C = 0;
    static uint8_t lastVal = 6;

    if (reset)
    {
        lastI2C = 0;
        lastVal = 6;
        return;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastI2C < 10)
        return;
    lastI2C = currentMillis;

    uint8_t raw_val = 0;
    comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
    uint8_t val = raw_val & 0x0F;
    if (val == 0)
        val = lastVal;
    else
        lastVal = val;

    switch (val)
    {
    case 6:
        setMotors(130, 130);
        break;
    case 4:
        setMotors(130, 126);
        break;
    case 12:
        setMotors(135, 95);
        break;
    case 8:
        setMotors(150, 40);
        break;
    case 2:
        setMotors(126, 130);
        break;
    case 3:
        setMotors(95, 135);
        break;
    case 1:
        setMotors(40, 150);
        break;
    case 15:
        setMotors(120, 120);
        break;
    default:
        setMotors(130, 130);
        break;
    }
}