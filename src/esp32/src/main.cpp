#include <Arduino.h>
#include "motor.h"
#include "steering.h"

// Current commanded values
static int currentSpeed = 0;    // -100..100
static int currentSteer = 90;   // 0..180
static int currentDirection = STOP;

// Brief stop when reversing direction to let back-EMF dissipate
#define DIRECTION_CHANGE_DELAY_MS 30

// Timing
static unsigned long lastCommandTime = 0;
static unsigned long lastStatusTime = 0;
static bool watchdogTripped = false;

// Line buffer for Pi commands
static char cmdBuf[64];
static uint8_t cmdLen = 0;

// Apply motor speed only when it actually changes
static void applyMotor(int speed)
{
  int newDirection = (speed > 0) ? FORWARD : (speed < 0) ? BACKWARD : STOP;

  // Back-EMF protection on direction reversal
  if ((currentDirection == FORWARD && newDirection == BACKWARD) ||
      (currentDirection == BACKWARD && newDirection == FORWARD)) {
    motor_stop();
    delay(DIRECTION_CHANGE_DELAY_MS);
  }

  currentDirection = newDirection;

  if (speed > 0) {
    motor_forward((uint8_t)speed);
  } else if (speed < 0) {
    motor_backward((uint8_t)(-speed));
  } else {
    motor_stop();
  }
}

// Parse and execute a complete command line
void handleCommand(const char* line)
{
  lastCommandTime = millis();
  watchdogTripped = false;

  if (line[0] == 'C' && line[1] == ':') {
    // C:<speed>,<steer>\n
    int speed = 0;
    int steer = 90;
    if (sscanf(line + 2, "%d,%d", &speed, &steer) == 2) {
      // Clamp values
      if (speed < -100) speed = -100;
      if (speed > 100) speed = 100;
      if (steer < 0) steer = 0;
      if (steer > 180) steer = 180;

      // Only apply motor/steering when values actually change
      if (speed != currentSpeed) {
        applyMotor(speed);
        currentSpeed = speed;
      }

      if (steer != currentSteer) {
        steering_set(steer);
        currentSteer = steer;
      }

    } else {
      Serial.printf("[ERR] Bad C cmd: %s\n", line);
    }

  } else if (line[0] == 'E' && (line[1] == '\0' || line[1] == '\n')) {
    // E\n — emergency stop
    motor_stop();
    steering_center();
    currentSpeed = 0;
    currentDirection = STOP;
    currentSteer = STEERING_CENTER;

  } else if (line[0] == 'R' && (line[1] == '\0' || line[1] == '\n')) {
    // R\n — reset encoder
    encoder_reset();

  } else {
    Serial.printf("[ERR] Unknown cmd: %s\n", line);
  }
}

void setup()
{
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(100);
  }

  Serial.println("ESP32-S3 Robot Controller Starting...");

  // Pi UART on GPIO 44(RX) / 43(TX)
  Serial1.begin(115200, SERIAL_8N1, PI_RX, PI_TX);

  motor_init();
  steering_init();

  lastCommandTime = millis();
  lastStatusTime = millis();

  Serial.println("Ready.");
}

void loop()
{
  unsigned long now = millis();

  // --- Read commands from Pi (Serial1) ---
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;
    }
  }

  // Re-read time AFTER processing commands.
  // handleCommand() sets lastCommandTime = millis(), which can be newer
  // than `now` captured at loop top. Without refreshing, unsigned
  // subtraction wraps around and falsely triggers the watchdog.
  now = millis();

  // --- Watchdog: stop motor if no command within timeout ---
  if (!watchdogTripped && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    motor_stop();
    currentSpeed = 0;
    currentDirection = STOP;
    watchdogTripped = true;
    Serial.println("[WDG] No command — motor stopped");
  }

  // --- Status reporting to Pi (~50Hz) ---
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;
    long enc = encoder_read();
    Serial1.printf("S:%ld,%d,%d\n", enc, currentSpeed, currentSteer);
  }
}
