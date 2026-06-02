#include "mpu.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroZ_offset = 0;
float current_angle = 0.0;
float target_angle = 0.0;
unsigned long prev_time = 0;
float dt_global = 0.0;
float current_gyro_rate = 0.0;

void Init_MPU(int sda_pin, int scl_pin) {
    Wire.begin(sda_pin, scl_pin);
    if (!mpu.begin()) {
        Serial.println("Lỗi MPU6050!");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
    calibrateMPU();
    prev_time = micros(); 
}

void calibrateMPU() {
    float sum = 0;
    int num_samples = 500;
    Serial.println("[IMU] Đang đo Offset Gyro Z... GIỮ YÊN ROBOT!");
    for (int i = 0; i < num_samples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sum += (g.gyro.z * 57.2958);
        delay(3);
    }
    gyroZ_offset = sum / num_samples;
    Serial.printf("[IMU] Offset: %.2f deg/s\n", gyroZ_offset);
}

float normalizeAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

float calculateAngleError(float target, float current) {
    return normalizeAngle(target - current);
}

void updateAngle() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long current_time = micros();
    dt_global = (current_time - prev_time) / 1000000.0;
    if (dt_global <= 0.001) return;
    prev_time = current_time;

    float gyroZ_rate = (g.gyro.z * 57.2958) - gyroZ_offset;
    current_gyro_rate = gyroZ_rate;

    if (abs(gyroZ_rate) > 0.3) {  
        current_angle += gyroZ_rate * dt_global;
        current_angle = normalizeAngle(current_angle); 
    }
}   