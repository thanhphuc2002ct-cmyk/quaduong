#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "master.h"
#include "mpu.h"
#include "bsp_periph.h"
#include "modes.h"

Master comm;
Peripheral periph;

enum AppMode { MODE_IDLE, MODE_MAZE, MODE_OBSTACLE, MODE_PICK, MODE_CROSSROAD, MODE_BROKEN_LINE };
AppMode currentMode = MODE_IDLE;

void setup() {
    Serial.begin(115200);
    comm.beginI2C(SDA_PIN, SCL_PIN);
    motorInit(); 
    Init_MPU(MPU_SDA_PIN, MPU_SCL_PIN); 
    comm.beginUART(Serial1, 41, 42, 115200);
    periph.Periph_Initialize(IR|servo|WS2812B, INIT);
    Serial.println("[SYSTEM] He thong khoi dong. Cho lenh tu remote IR...");
}

void loop() {
    int irKey = periph.Get_IR_Code();
    if (irKey != 0) {
        setMotors(0, 0); 
        if (irKey == '1') { currentMode = MODE_MAZE; modeMazeSolver(true); Serial.println("-> Chon Mode 1: Giai me cung"); }
        else if (irKey == '2') { currentMode = MODE_OBSTACLE; modeObstacleAvoidance(true); Serial.println("-> Chon Mode 2: Ne vat can"); }
        else if (irKey == '3') { currentMode = MODE_PICK; modePickAndDrop(true); Serial.println("-> Chon Mode 3: Gap tha vat"); }
        else if (irKey == '4') { currentMode = MODE_CROSSROAD; modeCrossroad(true); Serial.println("-> Chon Mode 4: Qua duong"); }
        else if (irKey == '5') { currentMode = MODE_BROKEN_LINE; modeBrokenLine(true); Serial.println("-> Chon Mode 5: Line dut quang"); }
        else if (irKey == 'O') { currentMode = MODE_IDLE; Serial.println("-> NGAT HE THONG (IDLE MODE)"); }
    }

    switch (currentMode) {
        case MODE_MAZE:         modeMazeSolver(false); break;
        case MODE_OBSTACLE:     modeObstacleAvoidance(false); break;
        case MODE_PICK:         modePickAndDrop(false); break;
        case MODE_CROSSROAD:    modeCrossroad(false); break;
        case MODE_BROKEN_LINE:  modeBrokenLine(false); break;
        default:                break;
    }
}