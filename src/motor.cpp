#include <Arduino.h>
#include "config.h"
#include "motor.h"

// Truyen PWM vao cac kenh LEDC tuong ung de dieu khien toc do motor
void setMotors(int speedLeft, int speedRight)
{
    speedLeft = speedLeft * SPEED_SCALE;
    speedRight = speedRight * SPEED_SCALE;

    if (speedLeft > 0)
    {
        speedLeft -= 0;
        if (speedLeft < 0)
            speedLeft = 0;
    }
    else if (speedLeft < 0)
    {
        speedLeft += 0;
        if (speedLeft > 0)
            speedLeft = 0;
    }

    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);
    
    // --- ĐẢO CHIỀU MOTOR TRÁI ---
    if (speedLeft > 0) {
        ledcWrite(0, 0);                 // Đưa về 0
        ledcWrite(1, speedLeft);         // Xuất PWM ở kênh 1 thay vì kênh 0
    } else if (speedLeft < 0) {
        ledcWrite(0, abs(speedLeft));    // Xuất PWM ở kênh 0 thay vì kênh 1
        ledcWrite(1, 0);                 // Đưa về 0
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }

    // --- ĐẢO CHIỀU MOTOR PHẢI ---
    if (speedRight > 0) {
        ledcWrite(2, 0);                 // Đưa về 0
        ledcWrite(3, speedRight);        // Xuất PWM ở kênh 3 thay vì kênh 2
    } else if (speedRight < 0) {
        ledcWrite(2, abs(speedRight));   // Xuất PWM ở kênh 2 thay vì kênh 3
        ledcWrite(3, 0);                 // Đưa về 0
    } else {
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}
// Khoi tao LEDC API de ep tan so PWM 20kHz cho ESP32
void motorInit()
{
    ledcSetup(0, 20000, 8);
    ledcSetup(1, 20000, 8);
    ledcSetup(2, 20000, 8);
    ledcSetup(3, 20000, 8);
    ledcAttachPin(M3_INA1, 0);
    ledcAttachPin(M3_INA2, 1);
    ledcAttachPin(M4_INA1, 2);
    ledcAttachPin(M4_INA2, 3);
    setMotors(0, 0);
}