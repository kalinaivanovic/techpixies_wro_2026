#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Motor pins (M1 on Romeo ESP32-S3)
#define MOTOR_EN 12
#define MOTOR_PN 13

// Encoder pins (on IO header)
#define ENCODER_A 44  // Green wire
#define ENCODER_B 43  // Yellow wire

// Direction definitions
#define FORWARD 1
#define BACKWARD 2
#define STOP 0

// Encoder data
extern volatile long encoderCount;
extern volatile int lastEncoded;
extern bool motorRunning;

void motor_init();
void motor_forward(uint8_t speed);
void motor_backward(uint8_t speed);
void motor_stop();
void motor_set(int8_t direction, uint8_t speed);

// Encoder functions
void encoder_init();
long encoder_read();
void encoder_reset();
bool check_stall();

#endif
