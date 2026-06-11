#ifndef PID_H
#define PID_H

#include <Arduino.h>

struct PIDConfig {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
};

extern PIDConfig pidStraight;
extern PIDConfig pidTurn;


// PWM tối thiểu để động cơ có thể nhích (bạn có thể tinh chỉnh lại)
#define MIN_PWM 50 

void driveWithHeading(int base_speed, float target, float current, PIDConfig &pid);


#endif