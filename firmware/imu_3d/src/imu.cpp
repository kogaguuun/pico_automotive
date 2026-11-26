#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "imu.h"

ImuReport imuReport = {0};
BNO08x myIMU;

/* Auto tare (2026-11-26, Koga's request): while cornering the yaw
   keeps tracking, but once the yaw stays within AUTO_TARE_TOL_DEG
   for AUTO_TARE_STILL_MS (i.e. running straight / standing still),
   the yaw is re-zeroed automatically. Only the yaw is zeroed:
   pitch and roll are preserved, so sitting on a slope does not
   erase the incline. */
static const float AUTO_TARE_TOL_DEG = 2.0f;
static const unsigned long AUTO_TARE_STILL_MS = 2000;

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

  /* Auto tare: if the reported yaw has stayed within the tolerance
     band for the full still period, re-zero the yaw. The base is
     rebuilt as base = tilt^-1 * current, where tilt is the current
     relative pitch/roll only (qy(pitch) * qx(roll)). The relative
     orientation therefore keeps its pitch/roll and its yaw becomes
     exactly 0. Unlike the button tare above, no +125 deg fixed
     offset is applied. (The button path is left untouched; its
     +125 deg offset is a presumed mounting correction whose meaning
     is still unverified -- see docs/implementation-guide.md,
     experiment 4.) */
  static float anchorYaw = 0;
  static unsigned long stillSinceMs = 0;
  static bool anchorValid = false;

  float dy = imuReport.yaw - anchorYaw;
  if (dy > 180.0f) dy -= 360.0f;
  if (dy < -180.0f) dy += 360.0f;

  unsigned long nowMs = millis();
  if (!anchorValid || fabsf(dy) > AUTO_TARE_TOL_DEG) {
    anchorYaw = imuReport.yaw;
    stillSinceMs = nowMs;
    anchorValid = true;
  } else if (nowMs - stillSinceMs >= AUTO_TARE_STILL_MS) {
    float hp = imuReport.pitch * PI / 360.0f; /* half angle, rad */
    float hr = imuReport.roll * PI / 360.0f;
    float cp = cos(hp), sp = sin(hp), cr = cos(hr), sr = sin(hr);
    /* tilt = qy(pitch) * qx(roll) = (cp*cr, cp*sr, sp*cr, -sp*sr);
       pass its inverse (conjugate) as the left factor. */
    float nbw, nbx, nby, nbz;
    quatMultiply(cp * cr, -(cp * sr), -(sp * cr), -(-sp * sr),
                 qw, qx, qy, qz, nbw, nbx, nby, nbz);
    baseQw = nbw;
    baseQx = nbx;
    baseQy = nby;
    baseQz = nbz;
    anchorYaw = 0; /* yaw reads 0 from the next report on */
    stillSinceMs = nowMs;
  }
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
