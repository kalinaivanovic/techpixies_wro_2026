#include "steering.h"
#include <ESP32Servo.h>

static Servo steeringServo;
static int currentAngle = STEERING_CENTER;

void steering_init()
{
  // Set pin to known state before servo takes over
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  delay(50);

  steeringServo.attach(SERVO_PIN, 500, 2500);  // Min/max pulse width from servo spec
  delay(50);
  steering_center();
  delay(100);  // Wait for servo to reach center
}

void steering_set(int angle)
{
  if (angle < STEERING_MIN) angle = STEERING_MIN;
  if (angle > STEERING_MAX) angle = STEERING_MAX;
  currentAngle = angle;
  steeringServo.write(angle);
}

void steering_left()
{
  steering_set(STEERING_LEFT);
}

void steering_right()
{
  steering_set(STEERING_RIGHT);
}

void steering_center()
{
  steering_set(STEERING_CENTER);
}
