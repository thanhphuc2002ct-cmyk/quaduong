#include "motor.h"
#include "config.h"

volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;

void IRAM_ATTR isr_encoder_left_a() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encoderLeftCount--; else encoderLeftCount++; }
void IRAM_ATTR isr_encoder_left_b() { if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encoderLeftCount++; else encoderLeftCount--; }
void IRAM_ATTR isr_encoder_right_a() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encoderRightCount++; else encoderRightCount--; }
void IRAM_ATTR isr_encoder_right_b() { if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encoderRightCount--; else encoderRightCount++; }

void encoderInit() {
    pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_encoder_left_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), isr_encoder_left_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_encoder_right_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), isr_encoder_right_b, CHANGE);
}

long getEncoderCount() { return (abs(encoderLeftCount) + abs(encoderRightCount)) / 2; }
void resetEncoders() { encoderLeftCount = 0; encoderRightCount = 0; }

void setMotors(int speedLeft, int speedRight) {
    // Thu phóng toàn bộ tốc độ truyền vào theo hệ số trước khi xử lý
    speedLeft = speedLeft * SPEED_SCALE;
    speedRight = speedRight * SPEED_SCALE;

    if (speedLeft > 0) { speedLeft -= 15; if (speedLeft < 0) speedLeft = 0; }
    else if (speedLeft < 0) { speedLeft += 15; if (speedLeft > 0) speedLeft = 0; }

    speedLeft = constrain(speedLeft, -255, 255);
    speedRight = constrain(speedRight, -255, 255);

    if (speedLeft > 0) { analogWrite(M3_INA1, 255 - speedLeft); analogWrite(M3_INA2, 255); }
    else if (speedLeft < 0) { analogWrite(M3_INA1, 255); analogWrite(M3_INA2, 255 + speedLeft); }
    else { analogWrite(M3_INA1, 0); analogWrite(M3_INA2, 0); }

    if (speedRight > 0) { analogWrite(M4_INA1, 255 - speedRight); analogWrite(M4_INA2, 255); }
    else if (speedRight < 0) { analogWrite(M4_INA1, 255); analogWrite(M4_INA2, 255 + speedRight); }
    else { analogWrite(M4_INA1, 0); analogWrite(M4_INA2, 0); }
}

void motorInit() {
    encoderInit();
    pinMode(M3_INA1, OUTPUT); pinMode(M3_INA2, OUTPUT);
    pinMode(M4_INA1, OUTPUT); pinMode(M4_INA2, OUTPUT);
    setMotors(0, 0);
}