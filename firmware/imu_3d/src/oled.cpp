#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "oled.h"

/*
   OLED表示バージョン (feature-muse/imu-oled-display)
   レイアウト (2026-11-24 確定, ui-mockups/imu_car_gauge_split.png):
   上段左: "IMU" / 上段右: ヨー数値 (Y+35 形式)
   中央:   相対クォータニオンで回すワイヤーフレーム車体
   最下段: P / R のセンターゼロゲージを左右分割 (中心が0, レンジ ±45°)
   姿勢の0リセットは imu.cpp 側の既存実装 (BTN_PIN 押下で基準姿勢を記録) を使う。
   ここでは相対化済みの imuReport (yaw/pitch/roll と クォータニオン) を描くだけ。
   
   描画は最大20fpsに制限する。I2C転送がボトルネックで、毎ループ全画面転送すると
   IMU読み取り側が痩せるため。平滑化は毎回かける。
*/

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* --- 表示用の平滑化 (オイラー角は従来どおり) --- */
static float fyaw = 0;
static float fpitch = 0;
static float froll = 0;
static const float alpha = 0.05f;

/* --- 車体の回転用にクォータニオンも nlerp で平滑化 (ヨーの±180°折り返しに強い) --- */
static float sqw = 1, sqx = 0, sqy = 0, sqz = 0;
static bool quatInit = false;
static const float quatAlpha = 0.15f;

/* --- 表示の視点 (画角): 姿勢ゼロのとき車の写真でよくある
   フロント 3/4 (前方左・少し上から) に見えるよう、固定の視点回転を
   姿勢クォータニオンへ前掛けする。サイト側シミュレータのカメラ位置
   (前方左・上) から求めた定数で、両者で同じ値を使う --- */
static const float VIEW_W = 0.41351f;
static const float VIEW_X = 0.05375f;
static const float VIEW_Y = 0.11715f;
static const float VIEW_Z = -0.90133f;
static float dqw = 1, dqx = 0, dqy = 0, dqz = 0;

static void composeViewQuat() {
  /* dq = VIEW (x) sq (ハミルトン積) */
  dqw = VIEW_W * sqw - VIEW_X * sqx - VIEW_Y * sqy - VIEW_Z * sqz;
  dqx = VIEW_W * sqx + VIEW_X * sqw + VIEW_Y * sqz - VIEW_Z * sqy;
  dqy = VIEW_W * sqy - VIEW_X * sqz + VIEW_Y * sqw + VIEW_Z * sqx;
  dqz = VIEW_W * sqz + VIEW_X * sqy - VIEW_Y * sqx + VIEW_Z * sqw;
}

static void smoothQuat() {
  float tw = imuReport.qw, tx = imuReport.qx, ty = imuReport.qy, tz = imuReport.qz;
  if (!quatInit) {
    sqw = tw; sqx = tx; sqy = ty; sqz = tz;
    quatInit = true;
    return;
  }
  float dot = sqw * tw + sqx * tx + sqy * ty + sqz * tz;
  if (dot < 0) {  /* 最短経路を取る */
    tw = -tw; tx = -tx; ty = -ty; tz = -tz;
  }
  sqw += (tw - sqw) * quatAlpha;
  sqx += (tx - sqx) * quatAlpha;
  sqy += (ty - sqy) * quatAlpha;
  sqz += (tz - sqz) * quatAlpha;
  float n = sqrt(sqw * sqw + sqx * sqx + sqy * sqy + sqz * sqz);
  if (n > 0.0001f) {
    sqw /= n; sqx /= n; sqy /= n; sqz /= n;
  }
}

/* --- 車体モデル: 側面シルエット (x=前後, z=上下) を y=幅方向へ押し出し --- */
static const float SIL_X[] = {-38, -38, -30, -16, 2, 16, 30, 38, 38};
static const float SIL_Z[] = {4, 14, 17, 26, 28, 21, 16, 11, 4};
static const int SIL_N = 9;
static const float HALF_W = 11;  /* 車幅の半分 (y) */
static const float WHEEL_X[] = {-24, 24};  /* 前後アクスルの x */
static const float WHEEL_Z = 9;  /* 車軸の z */
static const float WHEEL_R = 9;
static const int WHEEL_SEG = 12;

/* 画面配置: 車の表示中心とスケール (モック確定値) */
static const float CAR_CX = 64;
static const float CAR_CY = 27;
static const float CAR_CZ = 14;  /* モデルzの中心 (投影時に引く) */
static const float CAR_SCALE = 0.85f;

/*
   クォータニオンで点を回転し、正投影でスクリーン座標へ
   描画クォータニオン dq (視点回転込み) を使うので、姿勢ゼロでは
   フロント 3/4 の画角になる
*/
static void projectPoint(float x, float y, float z, int16_t &sx, int16_t &sy) {
  /* v' = v + 2 * cross(qv, cross(qv, v) + qw * v) */
  float cx1 = dqy * z - dqz * y;
  float cy1 = dqz * x - dqx * z;
  float cz1 = dqx * y - dqy * x;
  float tx = cx1 + dqw * x;
  float ty = cy1 + dqw * y;
  float tz = cz1 + dqw * z;
  float vx = x + 2 * (dqy * tz - dqz * ty);
  float vz = z + 2 * (dqx * ty - dqy * tx);
  sx = (int16_t)lroundf(CAR_CX + vx * CAR_SCALE);
  sy = (int16_t)lroundf(CAR_CY - (vz - CAR_CZ) * CAR_SCALE);
}

static void drawCar() {
  int16_t x0, y0, x1, y1;
  /* シルエット両側面 + 押し出しエッジ */
  for (int side = 0; side < 2; side++) {
    float y = (side == 0) ? -HALF_W : HALF_W;
    for (int i = 0; i < SIL_N; i++) {
      int j = (i + 1) % SIL_N;
      projectPoint(SIL_X[i], y, SIL_Z[i], x0, y0);
      projectPoint(SIL_X[j], y, SIL_Z[j], x1, y1);
      display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
    }
  }
  for (int i = 0; i < SIL_N; i++) {
    projectPoint(SIL_X[i], -HALF_W, SIL_Z[i], x0, y0);
    projectPoint(SIL_X[i], HALF_W, SIL_Z[i], x1, y1);
    display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  }
  /* タイヤ: 両側面に円 (多角形近似、回転で楕円になる) */
  for (int w = 0; w < 2; w++) {
    for (int side = 0; side < 2; side++) {
      float y = (side == 0) ? -HALF_W : HALF_W;
      int16_t px0, py0, px1, py1;
      for (int k = 0; k < WHEEL_SEG; k++) {
        float a0 = 2 * PI * k / WHEEL_SEG;
        float a1 = 2 * PI * (k + 1) / WHEEL_SEG;
        projectPoint(WHEEL_X[w] + WHEEL_R * cos(a0), y, WHEEL_Z + WHEEL_R * sin(a0), px0, py0);
        projectPoint(WHEEL_X[w] + WHEEL_R * cos(a1), y, WHEEL_Z + WHEEL_R * sin(a1), px1, py1);
        display.drawLine(px0, py0, px1, py1, SSD1306_WHITE);
      }
    }
  }
}

/* センターゼロゲージ: トラック x0..x1, 中心 cx (2px), 値は ±45° でフルスケール */
static void drawGauge(const char *label, float value, int16_t lx, int16_t x0, int16_t x1, int16_t cx) {
  const int16_t ty = 59;
  display.setCursor(lx, 56);
  display.print(label);
  display.drawFastHLine(x0, ty, x1 - x0 + 1, SSD1306_WHITE);
  display.drawFastVLine(x0, ty - 2, 5, SSD1306_WHITE);
  display.drawFastVLine(x1, ty - 2, 5, SSD1306_WHITE);
  display.drawFastVLine(cx, ty - 3, 7, SSD1306_WHITE);
  display.drawFastVLine(cx + 1, ty - 3, 7, SSD1306_WHITE);

  float v = constrain(value, -45.0f, 45.0f);
  int pos = (int)lroundf(v / 45.0f * (x1 - (cx + 1)));
  if (pos > 0) {
    display.fillRect(cx + 1, ty - 1, pos + 1, 3, SSD1306_WHITE);
  } else if (pos < 0) {
    display.fillRect(cx + pos, ty - 1, -pos + 1, 3, SSD1306_WHITE);
  } else {
    display.fillRect(cx - 1, ty - 1, 4, 3, SSD1306_WHITE);
  }
}

void updateOLED() {
  if (!imuReport.valid) {
    return;
  }

  fyaw = fyaw * (1 - alpha) + imuReport.yaw * alpha;
  fpitch = fpitch * (1 - alpha) + imuReport.pitch * alpha;
  froll = froll * (1 - alpha) + imuReport.roll * alpha;
  smoothQuat();
  composeViewQuat();

  /* 描画は最大20fps */
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 50) {
    return;
  }
  lastDraw = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  /* 上段: IMU ラベル (左) / ヨー数値 (右詰め) */
  display.setCursor(2, 0);
  display.print("IMU");
  char buf[12];
  snprintf(buf, sizeof(buf), "Y%+d", (int)lroundf(fyaw));
  int16_t textW = (int16_t)(strlen(buf) * 6);
  display.setCursor(SCREEN_WIDTH - textW, 0);
  display.print(buf);

  /* 中央: ワイヤーフレーム車体 */
  drawCar();

  /* 最下段: P (左半分) / R (右半分) のセンターゼロゲージ */
  drawGauge("P", fpitch, 1, 10, 61, 35);
  drawGauge("R", froll, 65, 74, 126, 100);

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

/* Nothing needed here for non-RTOS mode. */
