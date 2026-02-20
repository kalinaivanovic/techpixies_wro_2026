#include <Arduino.h>
#include "motor.h"
#include "steering.h"

// Current commanded values (for status reporting)
static int currentSpeed = 0;    // -100..100
static int currentSteer = 90;   // 0..180
static int currentDirection = STOP;   // Track direction for back-EMF protection

// Brief stop when reversing direction to let back-EMF dissipate
#define DIRECTION_CHANGE_DELAY_MS 30

// Timing
static unsigned long lastCommandTime = 0;
static unsigned long lastStatusTime = 0;
static bool watchdogTripped = false;

// Line buffer for Pi commands
static char cmdBuf[64];
static uint8_t cmdLen = 0;

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

      currentSpeed = speed;
      currentSteer = steer;

      // Apply steering
      steering_set(steer);

      // Apply motor (with back-EMF protection on direction reversal)
      int newDirection = (speed > 0) ? FORWARD : (speed < 0) ? BACKWARD : STOP;

      if ((currentDirection == FORWARD && newDirection == BACKWARD) ||
          (currentDirection == BACKWARD && newDirection == FORWARD)) {
        // Direction reversal — stop first to let back-EMF dissipate
        motor_stop();
        delay(DIRECTION_CHANGE_DELAY_MS);
        Serial.println("[MOT] Direction change — brief stop");
      }

      currentDirection = newDirection;

      if (speed > 0) {
        motor_forward((uint8_t)speed);
      } else if (speed < 0) {
        motor_backward((uint8_t)(-speed));
      } else {
        motor_stop();
      }

      // Debug print only on value change to avoid flooding USB serial
      static int lastPrintSpeed = -999;
      static int lastPrintSteer = -999;
      if (speed != lastPrintSpeed || steer != lastPrintSteer) {
        Serial.printf("[CMD] speed=%d steer=%d\n", speed, steer);
        lastPrintSpeed = speed;
        lastPrintSteer = steer;
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
    Serial.println("[CMD] Emergency stop");

  } else if (line[0] == 'R' && (line[1] == '\0' || line[1] == '\n')) {
    // R\n — reset encoder
    encoder_reset();
    Serial.println("[CMD] Encoder reset");

  } else {
    Serial.printf("[ERR] Unknown cmd: %s\n", line);
  }
}

void setup()
{
  // USB serial for debug
  Serial.begin(115200);
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(100);
  }

  Serial.println("ESP32-S3 Robot Controller Starting...");

  // Pi UART on GPIO 41(RX) / 42(TX)
  Serial1.begin(115200, SERIAL_8N1, PI_RX, PI_TX);

  Serial.println("Initializing motor...");
  motor_init();

  Serial.println("Initializing steering servo...");
  steering_init();

  lastCommandTime = millis();
  lastStatusTime = millis();

  Serial.println("Setup complete. Listening on Serial1 for Pi commands.");
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
      // Buffer overflow — discard
      cmdLen = 0;
    }
  }

  // --- Watchdog: stop motor if no command within timeout ---
  if (!watchdogTripped && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    motor_stop();
    currentSpeed = 0;
    currentDirection = STOP;
    watchdogTripped = true;
    Serial.println("[WDG] No command — motor stopped");
  }

  // --- Stall detection (disabled — encoder on GPIO 41/42 needs verification) ---
  // TODO: Re-enable once encoder wiring is confirmed reliable
  // if (check_stall()) {
  //   motor_stop();
  //   currentSpeed = 0;
  //   currentDirection = STOP;
  //   Serial1.println("E:STALL");
  //   Serial.println("[ERR] STALL DETECTED");
  // }

  // --- Status reporting to Pi (~50Hz) ---
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;
    long enc = encoder_read();
    Serial1.printf("S:%ld,%d,%d\n", enc, currentSpeed, currentSteer);
  }
}
