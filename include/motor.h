#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void motorInit();
void setMotors(int speedLeft, int speedRight);

void encoderInit();
long getEncoderCount();
long getEncoderLeft();
long getEncoderRight();
void resetEncoders();

#endif