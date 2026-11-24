// gas_ui1.ino — 燃料メーター UI案1「大きい数字+バー」版
//
// gas/gas.ino(Koga作・円形ゲージ版)のセンサ読み取りとポンプアイコン描画を
// そのまま使い、表示だけを 2026-11-23 採用のデザイン案1
// (大きい数字+バー+12セグメント)に差し替えた版。
// レイアウトは ui-mockups/ui1_native_icon.png の実測値ベース。
//
// ハードウェア:
//   ディスプレイ: SSD1306 128x64 I2C (SCL=GP5, SDA=GP4)
//   センサ入力: GP26 (ADC0)
//   ライブラリ: U8g2

#include <U8g2lib.h>
#include <Wire.h>

#define GP26 26

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE,
  5,  // SCL
  4   // SDA
);

// Koga作ポンプアイコン。gas.ino で円形ゲージ中央に描いていた図形プリミティブを
// そのまま関数化したもの(中心 cx, cy 基準、約19x19px)。
static void drawPumpIcon(U8G2 &u, int cx, int cy) {
  // 本体
  u.drawBox(cx - 6, cy - 10, 12, 18);
  u.drawBox(cx - 7, cy + 6, 14, 2);

  // 白抜き部分
  u.setDrawColor(0);
  u.drawBox(cx - 4, cy - 8, 8, 6);
  u.setDrawColor(1);

  // ホース
  u.drawLine(cx + 6, cy - 4, cx + 7, cy - 5);
  u.drawLine(cx + 8, cy - 4, cx + 8, cy + 2);
  u.drawLine(cx + 9, cy + 3, cx + 10, cy + 2);
  u.drawLine(cx + 10, cy + 2, cx + 10, cy - 6);

  // ノズル
  u.drawLine(cx + 10, cy - 7, cx + 11, cy - 7);
  u.drawLine(cx + 9, cy - 8, cx + 11, cy - 8);
  u.drawLine(cx + 8, cy - 9, cx + 10, cy - 9);
  u.drawLine(cx + 8, cy - 10, cx + 9, cy - 10);
}

// UI案1: 大きい数字+バー
static void drawFuelMeterUI1(U8G2 &u, int percent, int adc) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  u.clearBuffer();

  // 上段: FUEL ラベル / ポンプアイコン / ADC値
  u.setFont(u8g2_font_6x12_tr);
  u.drawStr(2, 11, "FUEL");
  drawPumpIcon(u, 55, 10);
  char buf[16];
  snprintf(buf, sizeof(buf), "ADC %d", adc);
  u.drawStr(128 - u.getStrWidth(buf), 11, buf);

  // 中段左: 大きい残量数字 + %
  u.setFont(u8g2_font_logisoso24_tn);
  snprintf(buf, sizeof(buf), "%d", percent);
  u.drawStr(6, 46, buf);
  int numW = u.getStrWidth(buf);
  u.setFont(u8g2_font_helvB12_tr);
  u.drawStr(6 + numW + 2, 46, "%");

  // 中段右: バーゲージ + E/F
  const int barX = 78, barY = 21, barW = 47, barH = 12;
  u.drawFrame(barX, barY, barW, barH);
  int fillW = percent * (barW - 2) / 100;
  if (fillW > 0) {
    u.drawBox(barX + 1, barY + 1, fillW, barH - 2);
  }
  u.setFont(u8g2_font_6x12_tr);
  u.drawStr(79, 47, "E");
  u.drawStr(118, 47, "F");

  // 下段: 12セグメント
  const int segY = 52, segW = 8, segH = 9;
  int lit = percent * 12 / 100;
  for (int i = 0; i < 12; i++) {
    int x = 4 + i * 10;
    if (i < lit) {
      u.drawBox(x, segY, segW, segH);
    } else {
      u.drawFrame(x, segY, segW, segH);
    }
  }

  u.sendBuffer();
}

void setup() {
  u8g2.begin();
  Serial.begin(115200);
}

void loop() {
  int raw = analogRead(GP26);  // GP26 (ADC0)
  Serial.println(raw);         // デバッグ用

  int percent = map(raw, 0, 1023, 0, 100);  // 第2引数、第3引数はセンサに合わせる

  drawFuelMeterUI1(u8g2, percent, raw);
  delay(20);
}
