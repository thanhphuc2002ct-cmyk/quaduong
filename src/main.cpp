#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "bsp_periph.h"
#include "modes.h"

Master comm;
Peripheral periph;
extern char remoteCmd;

enum AppMode
{
    MODE_IDLE,
    MODE_MAZE,
    MODE_OBSTACLE,
    MODE_PICK,
    MODE_CROSSROAD,
    MODE_BROKEN_LINE,
    MODE_REMOTE
};
AppMode currentMode = MODE_IDLE;
bool isRunning = false; // BIEN CO: false = Đứng im chờ lệnh, true = Cho phép chạy

long current_distance = 999; // Kéo biến siêu âm ra đây làm biến toàn cục thực sự

volatile bool btnPressed = false;
volatile unsigned long lastIntTime = 0;

void IRAM_ATTR buttonISR()
{
    unsigned long currentMillis = millis();
    if (currentMillis - lastIntTime > 200)
    {
        btnPressed = true;
        lastIntTime = currentMillis;
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit();
    Init_MPU(MPU_SDA_PIN, MPU_SCL_PIN);
    comm.beginUART(Serial1, 41, 42, 115200);
    periph.Periph_Initialize(IR | servo, INIT);
    extern void initCrossroadLED();
    initCrossroadLED();
    Serial.println("[SYSTEM] He thong khoi dong. Cho lenh tu remote IR...");
}

void loop()
{
    extern void updateAngle();
    updateAngle();
    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time >= 60)
    { // Chuẩn 60ms cho siêu âm
        last_debug_time = millis();
        extern long getSonarDistance();

        current_distance = getSonarDistance(); // Cập nhật thẳng vào biến toàn cục

        uint8_t raw_val = 0;
        comm.I2CrequestFrom(I2C_ADDR, 1, &raw_val);
        uint8_t current_line = raw_val & 0x0F;

        static long last_distance = -1;
        static uint8_t last_line = 255;
        static float last_angle = -999.0;

        if (current_distance != last_distance || current_line != last_line || abs(current_angle - last_angle) > 0.5)
        {
            Serial.printf("[DEBUG LOOP]   Line Sensor: %d  |  Goc IMU: %.2f deg\n", /*current_distance,*/ current_line, current_angle);
            last_distance = current_distance;
            last_line = current_line;
            last_angle = current_angle;
        }
    }
    if (btnPressed)
    {
        btnPressed = false;
        delay(15);
        if (digitalRead(BUTTON_PIN) == LOW)
        {
            unsigned long startH = millis();
            bool isLong = false;
            bool isCalib = false; // Thêm cờ nhận diện đè 10s

            while (digitalRead(BUTTON_PIN) == LOW)
            {
                extern void updateAngle();
                updateAngle();
                unsigned long heldTime = millis() - startH;

                if (heldTime >= 1000)
                {
                    isLong = true; // Đè qua 1s thì bật cờ chạy
                }
                delay(10);
            }

            if (isLong)
            {
                Serial.println("[BUTTON] DA DE 2S -> BAT DAU CHAY MODE HIEN TAI!");
                extern void triggerStartSequence();
                triggerStartSequence();

                while (digitalRead(BUTTON_PIN) == LOW)
                {
                    delay(10);
                }            // Chờ nhả nút
                delay(1000); // Thêm 1 giây delay để tay bạn rời hẳn khỏi xe, xe không bị rung lệch góc

                if (currentMode == MODE_MAZE)
                    modeMazeSolver(true);
                else if (currentMode == MODE_OBSTACLE)
                    modeObstacleAvoidance(true);
                else if (currentMode == MODE_PICK)
                    modePickAndDrop(true);
                else if (currentMode == MODE_CROSSROAD)
                    modeCrossroad(true);
                else if (currentMode == MODE_BROKEN_LINE)
                    modeBrokenLine(true);
                else if (currentMode == MODE_REMOTE)
                    modeRemoteControl(true);

                isRunning = true;
            }
            else
            {
                isRunning = false;
                setMotors(0, 0);
                extern void triggerModeChangeSequence();
                triggerModeChangeSequence();
                Serial.println("\n==================================");
                if (currentMode == MODE_OBSTACLE)
                {
                    currentMode = MODE_CROSSROAD;
                    modeCrossroad(true);
                    Serial.println("[BUTTON] CHUYEN MODE: QUA DUONG (CROSSROAD)");
                }
                else if (currentMode == MODE_CROSSROAD)
                {
                    currentMode = MODE_BROKEN_LINE;
                    modeBrokenLine(true);
                    Serial.println("[BUTTON] CHUYEN MODE: LINE DUT QUANG (BROKEN LINE)");
                }
                else
                {
                    currentMode = MODE_OBSTACLE;
                    modeObstacleAvoidance(true);
                    Serial.println("[BUTTON] CHUYEN MODE: NE VAT CAN (OBSTACLE)");
                }
                Serial.println("==================================\n");
            }
        }
    }

    int irKey = periph.Get_IR_Code();
    if (irKey != 0)
    {
        if (currentMode == MODE_REMOTE && (irKey == 'U' || irKey == 'D' || irKey == 'L' || irKey == 'R'))
        {
            remoteCmd = irKey;
        }
        else
        {
            isRunning = false; // Chuyển mode bằng remote cũng khóa đứng im
            setMotors(0, 0);
            if (irKey == '1')
            {
                currentMode = MODE_MAZE;
                modeMazeSolver(true);
                Serial.println("-> Chon Mode 1: Giai me cung");
            }
            else if (irKey == '2')
            {
                currentMode = MODE_OBSTACLE;
                modeObstacleAvoidance(true);
                Serial.println("-> Chon Mode 2: Ne vat can");
            }
            else if (irKey == '3')
            {
                currentMode = MODE_PICK;
                modePickAndDrop(true);
                Serial.println("-> Chon Mode 3: Gap tha vat");
            }
            else if (irKey == '4')
            {
                currentMode = MODE_CROSSROAD;
                modeCrossroad(true);
                Serial.println("-> Chon Mode 4: Qua duong");
            }
            else if (irKey == '5')
            {
                currentMode = MODE_REMOTE;
                modeRemoteControl(true);
                Serial.println("-> Chon Mode 5: Dieu khien Remote");
            }
            else if (irKey == '6')
            {
                currentMode = MODE_BROKEN_LINE;
                modeBrokenLine(true);
                Serial.println("-> Chon Mode 6: Line dut quang");
            }
            else if (irKey == '0')
            {
                currentMode = MODE_IDLE;
                Serial.println("-> NGAT HE THONG (IDLE MODE)");
            }
        }
    }

    if (isRunning)
    {
        switch (currentMode)
        {
        case MODE_MAZE:
            modeMazeSolver(false);
            break;
        case MODE_OBSTACLE:
            modeObstacleAvoidance(false);
            break;
        case MODE_PICK:
            modePickAndDrop(false);
            break;
        case MODE_CROSSROAD:
            modeCrossroad(false);
            break;
        case MODE_BROKEN_LINE:
            modeBrokenLine(false);
            break;
        case MODE_REMOTE:
            modeRemoteControl(false);
            break;
        default:
            break;
        }
    }
    else
    {
        setMotors(0, 0); // Đảm bảo xe bị khóa chết nếu chưa đè nút 2s
    }
}