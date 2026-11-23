#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "oled.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

static float fyaw = 0;
static float fpitch = 0;
static float froll = 0;
static const float alpha = 0.05f;

void updateOLED() {
  if (!imuReport.valid) {
    return;
  }

  fyaw = fyaw * (1 - alpha) + imuReport.yaw * alpha;
  fpitch = fpitch * (1 - alpha) + imuReport.pitch * alpha;
  froll = froll * (1 - alpha) + imuReport.roll * alpha;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Yaw:   ");
  display.println(fyaw, 1);

  display.setCursor(0, 16);
  display.print("Pitch: ");
  display.println(fpitch, 1);

  display.setCursor(0, 32);
  display.print("Roll:  ");
  display.println(froll, 1);

  display.display();
}

bool initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    return false;
  }
  display.setRotation(2);
  display.clearDisplay();
  display.display();
  return true;
}

// Nothing needed here for non-RTOS mode.
