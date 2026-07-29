#include "serial_output.h"
#include "common.h"

static unsigned long g_interval_ms = 10;
static unsigned long g_last_time = 0;

void serialOutputInit(unsigned long interval_ms) {
  g_interval_ms = interval_ms;
  g_last_time = millis();
}

void serialOutputLoop() {
  unsigned long now = millis();
  if (now - g_last_time >= g_interval_ms) {
    g_last_time = now;
    if (!imuReport.valid) return;

    // Primary packet: quaternion
    Serial.print("Q,");
    Serial.print(imuReport.qw, 6); Serial.print(',');
    Serial.print(imuReport.qx, 6); Serial.print(',');
    Serial.print(imuReport.qy, 6); Serial.print(',');
    Serial.println(imuReport.qz, 6);

    // Future: append other sensor fields here (temperature, accel, etc.)
  }
}
