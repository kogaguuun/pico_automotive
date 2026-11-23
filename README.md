# pico_automotive

Raspberry Pi Pico 系ボードを使った、車載向け組み込みプロジェクトのモノレポ。
YouTube チャンネル「KogaRen Drift」の別編（Raspberry Pi / 組み込みソフトウェア企画）の題材として開発している。

## 構成

| パス | 内容 | 状態 |
|---|---|---|
| `firmware/imu_3d/` | IMU（BNO086）の姿勢を読み取り、OLED 表示と USB シリアル出力をする Pico 2 向けファームウェア | 稼働中 |
| `firmware/fuel_meter/` | 燃料メーター（ADC → OLED ゲージ表示） | コード取り込み待ち |
| `viewer/` | ファームウェアのシリアル出力を受けて車両 3D モデルを動かす PC 側ビューア（three.js） | コード取り込み待ち |

## firmware/imu_3d

Raspberry Pi Pico 2（RP2350）＋ BNO086 IMU の姿勢データを、SSD1306 OLED へ表示しつつ
USB シリアルへ 100Hz で送り出すファームウェア。PC 側ビューア（`viewer/`）が
このクォータニオンを受けて車両 3D モデルを動かす想定。

### ビルド

PlatformIO を使用する。

```bash
cd firmware/imu_3d
pio run            # ビルド
pio run -t upload  # 書き込み（upload_protocol = picotool）
```

`platformio.ini` の内容（env: `rpipico2`）:

- platform: `maxgerhardt/platform-raspberrypi` の develop ブランチ（GitHub URL で直接指定）
- framework: Arduino
- ライブラリ依存:
  - `sparkfun/SparkFun BNO08x Cortex Based IMU`（BNO086 ドライバ）
  - `adafruit/Adafruit SSD1306` / `adafruit/Adafruit GFX Library`（OLED）

### 配線・ピン

いずれも I2C（Wire、100kHz）接続。ピン定義は `firmware/imu_3d/src/common.h` 参照。

| 部品 | 接続 |
|---|---|
| BNO086 IMU | I2C |
| SSD1306 OLED（128×64） | I2C、アドレス `0x3C`、画面は 180° 回転（`setRotation(2)`） |
| 基準姿勢ボタン | GPIO10（`BTN_PIN`、内部プルアップ、押下 = LOW） |
| IMU リセット | GPIO2（`RST_PIN`。ハードリセット関数 `hardResetBNO()` は定義のみで現状未使用） |

### シリアル出力仕様

- USB シリアル、115200 baud
- 送信間隔: 10ms（100Hz）。IMU データが有効なときのみ送信
- フォーマット: `Q,qw,qx,qy,qz`（各値は小数点以下 6 桁の CSV 1 行）

```text
Q,0.999123,0.012345,-0.023456,0.034567
```

- ボタンを押すと、その時点の姿勢を「基準姿勢」として記録し、以後は基準に対する
  相対クォータニオンと Yaw/Pitch/Roll を出力する（OLED には Yaw/Pitch/Roll を
  指数平滑して数値表示）。
- 基準姿勢には Z 軸 +125° の固定オフセットが加算されている（`src/imu.cpp`）。
  IMU の取り付け角度の補正とみられるが、コード内に理由の記述はなく意図は未確認。
