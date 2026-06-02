#ifndef MPU_H
#define MPU_H

#include <Arduino.h>

extern float current_angle;
extern float target_angle;
extern float dt_global;
extern float current_gyro_rate;

void Init_MPU(int sda_pin, int scl_pin);
void calibrateMPU();
float normalizeAngle(float angle);
float calculateAngleError(float target, float current);
void updateAngle();

#endif