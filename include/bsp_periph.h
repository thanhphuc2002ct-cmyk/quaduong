#ifndef BSP_PERIPH_H
#define BSP_PERIPH_H

#include "stdint.h"


typedef enum {
    IR = 0x01,
    WS2812B = 0x02,
    Buzzer = 0x03,
    servo = 0x04
}PeripheralID;

typedef enum {
    DEINIT = 0,
    INIT,
    CMD,
    DATA
}Peripheral_InitState;

/* Servo Pins Definitions */
typedef enum {
    SERVO_S1 = 0x01,
    SERVO_S2 = 0x02,
    SERVO_S3 = 0x03,
    SERVO_S4 = 0x04
}ServoID;

typedef enum {
    OFF = 0,
    ON
}RGB_State;

typedef enum {
    Key_1 = 0xA25D,
    Key_2 = 0x629D,
    Key_3 = 0xE21D,
    Key_4 = 0x22DD,
    Key_5 = 0x02FD,
    Key_6 = 0xC23D,
    Key_7 = 0xE01F,
    Key_8 = 0xA857,
    Key_9 = 0x906F,
    Key_0 = 0x9867,
    Key_Asterisk = 0x6897,
    Key_Hastag = 0xB04F,
    Key_Up = 0x18E7,
    Key_Down = 0x4AB5,
    Key_Left = 0x10EF,
    Key_Right = 0x5AA5,
    Key_OK = 0x38C7
}IRKey_Code17;

#define RGB_State uint8_t

class Peripheral {
    public:
        void Periph_Initialize(uint8_t peripheral_id, Peripheral_InitState state);
        void OutputControlRGB(RGB_State state);
        void Buzzer_On();
        void SetRGBColor(uint8_t r, uint8_t g, uint8_t b);
        void Servo_SetAngle(ServoID servo_id, uint8_t angle);
        uint8_t Get_IR_Code();
};

#endif // BSP_PERIPH_H