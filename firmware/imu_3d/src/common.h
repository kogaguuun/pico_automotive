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

#endif // COMMON_H
