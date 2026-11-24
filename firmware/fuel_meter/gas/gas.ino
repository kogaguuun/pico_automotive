#include <U8g2lib.h>
#include <Wire.h>

#define GP26 26
#define INIT 0

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  U8X8_PIN_NONE,
  5,  // SCL
  4   // SDA
);

void setup() {
  u8g2.begin();
  Serial.begin(115200);
}




void loop() {
  int raw = analogRead(GP26);  // GP26 (ADC0)
  Serial.println(raw);       // デバッグ用

  int percent = map(raw, 0, 1023, 0, 100); //第2引数、第3引数はセンサに合わせる

  u8g2.clearBuffer();

  // --- 円形ゲージ ---
  int cx = 64;
  int cy = 40;
  int r  = 25;

  // 外枠
  u8g2.drawCircle(cx, cy, r, U8G2_DRAW_ALL);
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,60,128,4);
  u8g2.setDrawColor(1);

  // 太いゲージ（3ドット幅）
  int angle = map(percent, 0, 100, 126, 414);

  for (int a = 126; a < angle; a++) {
    float rad = a * 3.14159 / 180.0;

    for (int w = 0; w < 3; w++) {  // 太さ3
      int x = cx + cos(rad) * (r - w);
      int y = cy + sin(rad) * (r - w);
      u8g2.drawPixel(x, y);
    }
  }

  // --- ガソリンマーク（中央） ---
  // 本体
  u8g2.drawBox(cx - 6, cy - 10, 12, 18); //x=56,y=30
  u8g2.drawBox(cx - 7, cy + 6, 14, 2);

  // 白抜き部分
  u8g2.setDrawColor(0);
  u8g2.drawBox(cx - 4, cy - 8, 8, 6);
  u8g2.setDrawColor(1);

  // ホース
  u8g2.drawLine(cx + 6, cy - 4, cx + 7, cy-5);
  u8g2.drawLine(cx + 8, cy - 4, cx + 8, cy + 2);
  u8g2.drawLine(cx + 9, cy + 3, cx + 10, cy + 2);
  u8g2.drawLine(cx + 10, cy + 2, cx + 10, cy - 6);

  //ノズル
  u8g2.drawLine(cx + 10, cy - 7, cx + 11, cy - 7);
  u8g2.drawLine(cx + 9, cy - 8, cx + 11, cy - 8);
  u8g2.drawLine(cx + 8, cy - 9, cx + 10, cy - 9);
  u8g2.drawLine(cx + 8, cy - 10, cx + 9, cy -10);

  // 数値
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.setCursor(0, 10);
  u8g2.print("gas:");
  u8g2.print(percent);
  u8g2.print("%");

  u8g2.setCursor(64, 10);
  u8g2.print("adc:");
  u8g2.print(raw);


  u8g2.setCursor(cx - r - 4, 62);
  u8g2.print("E");
  u8g2.setCursor(cx + r, 62);
  u8g2.print("F");

  u8g2.sendBuffer();
  delay(20);
}

