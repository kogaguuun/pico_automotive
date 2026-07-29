#include "common.h"
#include "imu.h"
#include "oled.h"
#include "serial_output.h"
#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  // Wait briefly for USB CDC to enumerate so early prints are not lost
  unsigned long _serial_wait_start = millis();
  while (!Serial && millis() - _serial_wait_start < 3000) {
    delay(10);
  }
  delay(2000);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(100000);

  if (!initOLED()) {
    Serial.println("OLED init failed");
    while (1);
  }

  if (!initIMU()) {
    Serial.println("IMU init failed");
    while (1);
  }

  // Initialize serial output at 10ms interval
  serialOutputInit(10);
}

void loop() {
  updateIMU();
  updateOLED();
  // Run serial output task (non-blocking). Sends every 10ms.
  serialOutputLoop();

  // Keep loop responsive; short sleep to let background tasks run.
  delay(1);
}
