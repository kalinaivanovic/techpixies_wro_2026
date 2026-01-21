#include <Arduino.h>
#include "motor.h"
#include "steering.h"

void setup()
{
  Serial.begin(115200);

  // Wait for serial connection
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(100);
  }

  Serial.println("ESP32-S3 Robot Controller Starting...");

  // Initialize motor
  Serial.println("Initializing motor...");
  motor_init();

  // Initialize steering servo
  Serial.println("Initializing steering servo...");
  steering_init();

  Serial.println("Setup complete. Waiting for commands from Raspberry Pi...");
}

void loop()
{
  // TODO: Implement communication protocol with Raspberry Pi
  // TODO: Implement watchdog - stop motors if no command received

  // Check for motor stall
  if (check_stall()) {
    Serial.println("STALL DETECTED - stopping motor!");
    motor_stop();
  }

  delay(10);
}
