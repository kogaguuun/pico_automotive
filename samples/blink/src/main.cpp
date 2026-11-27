#include <Arduino.h>

/* Pico 2 W のオンボード LED を1秒間隔で点滅させるだけのサンプル。
   環境構築の確認用（ビルドと書き込みの練習）なので、センサ類は不要。 */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
