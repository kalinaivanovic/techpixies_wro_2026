#ifndef STEERING_H
#define STEERING_H

#include <Arduino.h>

// Servo pin
#define SERVO_PIN 38

// Steering positions
#define STEERING_CENTER 90
#define STEERING_LEFT 10
#define STEERING_RIGHT 180
#define STEERING_MIN 0
#define STEERING_MAX 180

void steering_init();
void steering_set(int angle);
void steering_left();
void steering_right();
void steering_center();

#endif
