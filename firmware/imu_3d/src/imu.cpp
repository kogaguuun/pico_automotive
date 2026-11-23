#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "imu.h"

ImuReport imuReport = {0};
BNO08x myIMU;

static float baseQw = 1;
static float baseQx = 0;
static float baseQy = 0;
static float baseQz = 0;

static void hardResetBNO() {
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(50);
  digitalWrite(RST_PIN, HIGH);
  delay(200);
}

static void quatInverse(float w, float x, float y, float z,
                        float &iw, float &ix, float &iy, float &iz) {
  iw = w;
  ix = -x;
  iy = -y;
  iz = -z;
}

static void quatMultiply(float aw, float ax, float ay, float az,
                         float bw, float bx, float by, float bz,
                         float &rw, float &rx, float &ry, float &rz) {
  rw = aw*bw - ax*bx - ay*by - az*bz;
  rx = aw*bx + ax*bw + ay*bz - az*by;
  ry = aw*by - ax*bz + ay*bw + az*bx;
  rz = aw*bz + ax*by - ay*bx + az*bw;
}

static void updateIMUInternal() {
  if (!myIMU.getSensorEvent()) {
    return;
  }

  float qw = myIMU.getQuatReal();
  float qx = myIMU.getQuatI();
  float qy = myIMU.getQuatJ();
  float qz = myIMU.getQuatK();

  float norm = qw*qw + qx*qx + qy*qy + qz*qz;
  if (norm < 0.5f) {
    return;
  }

  if (digitalRead(BTN_PIN) == LOW) {
    baseQw = qw;
    baseQx = qx;
    baseQy = qy;
    baseQz = qz;

    float angle = +125.0 * PI / 180.0;
    float half = angle / 2.0;
    float fw = cos(half);
    float fx = 0;
    float fy = 0;
    float fz = sin(half);

    float rw, rx, ry, rz;
    quatMultiply(fw, fx, fy, fz,
                 baseQw, baseQx, baseQy, baseQz,
                 rw, rx, ry, rz);

    baseQw = rw;
    baseQx = rx;
    baseQy = ry;
    baseQz = rz;
  }

  float iw, ix, iy, iz;
  quatInverse(baseQw, baseQx, baseQy, baseQz, iw, ix, iy, iz);

  float cw, cx, cy, cz;
  quatMultiply(qw, qx, qy, qz, iw, ix, iy, iz, cw, cx, cy, cz);

  float cn = sqrt(cw*cw + cx*cx + cy*cy + cz*cz);
  if (cn <= 0.0001f) {
    return;
  }
  cw /= cn;
  cx /= cn;
  cy /= cn;
  cz /= cn;

  float yaw = atan2(2*(cw*cz + cx*cy), 1 - 2*(cy*cy + cz*cz));
  float sinp = 2*(cw*cy - cz*cx);
  sinp = constrain(sinp, -1, 1);
  float pitch = asin(sinp);
  float roll = atan2(2*(cw*cx + cy*cz), 1 - 2*(cx*cx + cy*cy));

  imuReport.yaw = yaw * 180.0 / PI;
  imuReport.pitch = pitch * 180.0 / PI;
  imuReport.roll = roll * 180.0 / PI;
  imuReport.qw = cw;
  imuReport.qx = cx;
  imuReport.qy = cy;
  imuReport.qz = cz;
  imuReport.valid = true;
}

bool initIMU() {
  Serial.println("BNO086 begin...");
  if (!myIMU.begin()) {
    Serial.println("begin FAILED");
    return false;
  }
  Serial.println("begin OK");

  bool ok = myIMU.enableReport(SH2_ROTATION_VECTOR, 5000);
  Serial.println(ok ? "enableReport OK" : "enableReport FAILED");
  return ok;
}

void updateIMU() {
  updateIMUInternal();
}
