#include "motor.h"
#include "config.h"

void motorInit() {
    pinMode(M3_INA1, OUTPUT);
    pinMode(M3_INA2, OUTPUT);
    pinMode(M4_INA1, OUTPUT);
    pinMode(M4_INA2, OUTPUT);
    setMotors(0, 0);
}

// Trim giảm 15 đơn vị PWM cho bánh trái để cân bằng phần cứng và giữ vạch giữa
void setMotors(int speedLeft, int speedRight) {
    if (speedLeft > 0) {
        speedLeft -= 10;
        if (speedLeft < 0) speedLeft = 0;
    } else if (speedLeft < 0) {
        speedLeft += 10;
        if (speedLeft > 0) speedLeft = 0;
    }

    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);

    if (speedLeft > 0) {
        analogWrite(M3_INA1, 255 - speedLeft);
        analogWrite(M3_INA2, 255); 
    } else if (speedLeft < 0) {
        analogWrite(M3_INA1, 255); 
        analogWrite(M3_INA2, 255 + speedLeft);
    } else {
        analogWrite(M3_INA1, 0); 
        analogWrite(M3_INA2, 0);
    }

    if (speedRight > 0) {
        analogWrite(M4_INA1, 255 - speedRight);
        analogWrite(M4_INA2, 255); 
    } else if (speedRight < 0) {
        analogWrite(M4_INA1, 255); 
        analogWrite(M4_INA2, 255 + speedRight);
    } else {
        analogWrite(M4_INA1, 0);
        analogWrite(M4_INA2, 0);
    }
}