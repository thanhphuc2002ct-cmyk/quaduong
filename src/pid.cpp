#include "pid.h"
#include "motor.h"
#include "mpu.h" 

// PIDConfig pidStraight = {2.5, 0.01, 0.1, 0, 0};  
// PIDConfig pidTurn     = {1.5, 0.04, 0.06, 0, 0};    

PIDConfig pidStraight = {0.6, 0.0, 0.15, 0, 0};
PIDConfig pidTurn     = {1.5, 0.02, 0.15, 0, 0};

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
    
if (base_speed == 0) {
        // Lực cản quét ngang của bánh mắt trâu phía trước rất lớn
        // Cần cộng thêm 20 đơn vị PWM (Castor Kick) vào MIN_PWM để thắng lực lết bánh
        int castor_deadband = MIN_PWM + 10; 
        
        if (abs(error) > 1.5) {
            if (error > 0) correction += castor_deadband;
            else correction -= castor_deadband;
        } else {
            correction = 0; 
        }
        
        // Nới trần tốc độ xoay lên 110 để xe không bị ghì lại bởi ma sát lết của bánh trước
        correction = constrain(correction, -110.0, 110.0);
    } else {
        float max_corr = abs(base_speed) * 0.4;
        correction = constrain(correction, -max_corr, max_corr);
    }

    float left_pwm = base_speed - correction;
    float right_pwm = base_speed + correction;

    // Chỉ giới hạn tỷ lệ PWM khi xe đang đi thẳng/lùi
    if (base_speed != 0) {
        float max_pwm_req = max(abs(left_pwm), abs(right_pwm));
        if (max_pwm_req > 255.0) {
            left_pwm = (left_pwm * 255.0) / max_pwm_req;
            right_pwm = (right_pwm * 255.0) / max_pwm_req;
        }
    }

    // Gọi hàm setMotors có sẵn trong motor.cpp của quaduong
    setMotors((int)left_pwm, (int)right_pwm);
}