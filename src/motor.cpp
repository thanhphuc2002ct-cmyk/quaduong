#include "motor.h"
#include "config.h"

void motorInit() {
    pinMode(M3_INA1, OUTPUT);
    pinMode(M3_INA2, OUTPUT);
    pinMode(M4_INA1, OUTPUT);
    pinMode(M4_INA2, OUTPUT);
    setMotors(0, 0);
}

void setMotors(int speedLeft, int speedRight) {
    // Động cơ trái (M3)
    if (speedLeft > 0) {
        analogWrite(M3_INA1, 255 - speedLeft);
        analogWrite(M3_INA2, 255); // Thay digitalWrite bằng analogWrite
    } else if (speedLeft < 0) {
        analogWrite(M3_INA1, 255); // Thay digitalWrite bằng analogWrite
        analogWrite(M3_INA2, 255 + speedLeft);
    } else {
        // Cúp điện hoàn toàn, thả trôi (Coast)
        analogWrite(M3_INA1, 0); 
        analogWrite(M3_INA2, 0);
    }

    // Động cơ phải (M4)
    if (speedRight > 0) {
        analogWrite(M4_INA1, 255 - speedRight);
        analogWrite(M4_INA2, 255); 
    } else if (speedRight < 0) {
        analogWrite(M4_INA1, 255); 
        analogWrite(M4_INA2, 255 + speedRight);
    } else {
        // Cúp điện hoàn toàn, thả trôi (Coast)
        analogWrite(M4_INA1, 0);
        analogWrite(M4_INA2, 0);
    }
}