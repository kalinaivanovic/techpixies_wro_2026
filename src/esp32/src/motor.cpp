#include "motor.h"
#include "driver/mcpwm.h"

// Encoder variables
volatile long encoderCount = 0;
volatile int lastEncoded = 0;
static long lastEncoderCheck = 0;
static unsigned long lastCheckTime = 0;

// Motor state
bool motorRunning = false;

// Encoder interrupt handler
void IRAM_ATTR encoderISR()
{
  int MSB = digitalRead(ENCODER_A);
  int LSB = digitalRead(ENCODER_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

  lastEncoded = encoded;
}

void encoder_init()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);
}

long encoder_read()
{
  return encoderCount;
}

void encoder_reset()
{
  encoderCount = 0;
}

bool check_stall()
{
  if (!motorRunning) {
    return false;  // Motor not running, no stall possible
  }

  if (millis() - lastCheckTime < 200) {
    return false;  // Check every 200ms
  }

  lastCheckTime = millis();
  long currentCount = encoderCount;

  if (currentCount == lastEncoderCheck) {
    // Motor should be moving but encoder shows no change
    lastEncoderCheck = currentCount;
    return true;  // Stall detected
  }

  lastEncoderCheck = currentCount;
  return false;
}

void motor_init()
{
  // First set pins to LOW to prevent motor from running during init
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_PN, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);
  digitalWrite(MOTOR_PN, LOW);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_EN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_PN);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Ensure motor is stopped after init
  motor_stop();

  encoder_init();
}

void motor_forward(uint8_t speed)
{
  motorRunning = true;
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);  // Swapped
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, speed);
}

void motor_backward(uint8_t speed)
{
  motorRunning = true;
  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);  // Swapped
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, speed);
}

void motor_stop()
{
  motorRunning = false;
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B);
}

void motor_set(int8_t direction, uint8_t speed)
{
  switch(direction)
  {
    case FORWARD:
      motor_forward(speed);
      break;
    case BACKWARD:
      motor_backward(speed);
      break;
    case STOP:
    default:
      motor_stop();
      break;
  }
}
