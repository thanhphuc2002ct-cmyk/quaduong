#include "pid.h"
#include "motor.h"
#include "mpu.h" 

// PIDConfig pidStraight = {2.5, 0.01, 0.1, 0, 0};  
// PIDConfig pidTurn     = {1.5, 0.04, 0.06, 0, 0};    

PIDConfig pidStraight = {0.5, 0.0, 0.1, 0, 0}; // PID cho di chuyển thẳng lùi
PIDConfig pidTurn     = {0.6, 0.01, 0.05, 0, 0};

void driveWithHeading(int base_speed, float target, float current, PIDConfig &pid) {
    float error = calculateAngleError(target, current);
    if (abs(error) < 20.0) {
        pid.integral += error * dt_global;
    } else {
        pid.integral = 0;
    }
    pid.integral = constrain(pid.integral, -50.0, 50.0); 

    float derivative = -current_gyro_rate;
    float correction = (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
    
    float left_pwm = base_speed - correction;
    float right_pwm = base_speed + correction;

    float max_pwm_req = max(abs(left_pwm), abs(right_pwm));
    if (max_pwm_req > 255.0) {
        left_pwm = (left_pwm * 255.0) / max_pwm_req;
        right_pwm = (right_pwm * 255.0) / max_pwm_req;
    }

    if (base_speed == 0 && abs(error) > 1.5) { 
        if (left_pwm > 0 && left_pwm < MIN_PWM) left_pwm = MIN_PWM;
        else if (left_pwm < 0 && left_pwm > -MIN_PWM) left_pwm = -MIN_PWM;
        if (right_pwm > 0 && right_pwm < MIN_PWM) right_pwm = MIN_PWM;
        else if (right_pwm < 0 && right_pwm > -MIN_PWM) right_pwm = -MIN_PWM;
    }

    // Gọi hàm setMotors có sẵn trong motor.cpp của quaduong
    setMotors((int)left_pwm, (int)right_pwm);
}