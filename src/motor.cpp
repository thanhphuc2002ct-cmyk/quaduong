#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include <Preferences.h>
Preferences prefs;
float leftTrim = 1.0;
float rightTrim = 1.0;
// Truyen PWM vao cac kenh LEDC tuong ung de dieu khien toc do motor
void setMotors(int speedLeft, int speedRight)
{
    // Áp dụng ngay hệ số cân bằng sau khi nhân hệ số Scale
    speedLeft = speedLeft * SPEED_SCALE * leftTrim;
    speedRight = speedRight * SPEED_SCALE * rightTrim;

    // Đoạn bù trừ ma sát (nếu bạn không dùng tới, có thể bỏ. Tạm thời mình giữ nguyên cấu trúc cũ của bạn)
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

    if (speedRight > 0)
    {
        speedRight -= 0;
        if (speedRight < 0)
            speedRight = 0;
    }
    else if (speedRight < 0)
    {
        speedRight+= 0;
        if (speedRight > 0)
            speedRight = 0;
    }
    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);
    
    // --- ĐẢO CHIỀU MOTOR TRÁI ---
    if (speedLeft > 0) {
        ledcWrite(0, 0);             
        ledcWrite(1, speedLeft);         
    } else if (speedLeft < 0) {
        ledcWrite(0, abs(speedLeft));   
        ledcWrite(1, 0);               
    } else {
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }

    // --- ĐẢO CHIỀU MOTOR PHẢI ---
    if (speedRight > 0) {
        ledcWrite(2, 0);                 
        ledcWrite(3, speedRight);        
    } else if (speedRight < 0) {
        ledcWrite(2, abs(speedRight));   
        ledcWrite(3, 0);                
    } else {
        ledcWrite(2, 0);
        ledcWrite(3, 0);
    }
}// Khoi tao LEDC API de ep tan so PWM 20kHz cho ESP32
void motorInit()
{
    prefs.begin("motor_data", false);
    
    // Lấy thông số đã lưu, nếu chưa từng calib bao giờ thì lấy mặc định là 1.0
    leftTrim = prefs.getFloat("left", 1.0);
    rightTrim = prefs.getFloat("right", 1.0);
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