#ifndef COMMON_H
#define COMMON_H

#include <Arduino.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BTN_PIN 10
#define RST_PIN 2

typedef struct {
  float yaw;
  float pitch;
  float roll;
  float qw;
  float qx;
  float qy;
  float qz;
  bool valid;
} ImuReport;

extern ImuReport imuReport;

/* GPS report, filled by gps.cpp from NMEA RMC/GGA sentences. */
/* NOTE: course_deg is true-north based (GPS), while ImuReport.yaw is */
/* relative to the tare orientation. They cannot be subtracted directly; */
/* the offset alignment is step 3 of docs/gps-logger-design.md. */
typedef struct {
  double lat_deg; /* + north */
  double lon_deg; /* + east */
  float speed_kmh; /* ground speed (RMC) */
  float course_deg; /* course over ground, 0-360, true north (RMC) */
  uint8_t hour;
  uint8_t minute;
  uint8_t second; /* UTC (RMC) */
  uint16_t year;
  uint8_t month;
  uint8_t day; /* UTC (RMC) */
  bool fix; /* RMC status == 'A' */
  uint8_t sats; /* satellites in use (GGA) */
  bool updated; /* set on each parsed RMC; consumer clears it */
  unsigned long last_fix_ms; /* millis() of the last valid fix */
} GpsReport;

extern GpsReport gpsReport;

#endif /* COMMON_H */
