# ファームウェア開発ノウハウ

MSPM0G3507 ファームウェア開発で知っておくべき情報。

---

## UART設定

### Yahboomボードのピン配置

| 用途 | UART | TX | RX | ボーレート |
|------|------|----|----|-----------|
| RPi通信（GPIO UART直結） | UART3 (UART_0) | PA26 | PA25 | 115200 |
| モータードライバ通信 | UART1 | PB6 | PB7 | 115200 |

**注意**: TI SDKサンプルはUART1を使用しているが、Yahboomボードの USB-シリアル(CH340) は **UART0 (PA10/PA11)** に接続されている。SDKサンプルはそのまま動かない。Yahboomのサンプルコードをベースにすること。

**RPi接続変更 (2026-02)**: USB(CH340)経由の接続はMCU電池からの逆流問題があるため、GPIO UART直結(PA25/PA26)に変更。ラズパイ側は `/dev/ttyAMA0` (GPIO14=TXD, GPIO15=RXD) を使用。電源線なし（TX/RX/GNDのみ）。

---

## BSL書き込み手順

### UniFlash設定

1. デバイス: **MSPM0G3507(BOOTLOADER) - Serial**
2. COM Port: **COM3**（環境による）
3. Baud Rate: **9600**
4. **Application Image 1** に `.hex` ファイルを設定
   - 「Password」欄ではない！

### BSLモード進入

1. **BSL**ボタンを押したまま
2. **NRST**ボタンを押して離す
3. 1秒待つ
4. **BSL**ボタンを離す
5. 10秒以内に **Load Image** をクリック

### エラー対処

| エラー | 対処 |
|--------|------|
| Error: -1 | 偶発的エラー。再度書き込む |
| Error: -3 | パスに日本語あり。英語パスに移動 |
| Error: -6 | COMポートが間違い、または占有中 |
| Error: -9 | BSLモードに入っていない |
| "Image loading failed: Try manual..." | 実際は成功の可能性あり（UniFlashのバグ） |

---

## CCS Theia 設定

### Hex出力の有効化

1. プロジェクト右クリック → **Properties**
2. **Build** → **Arm Hex Utility**
3. **Enable Arm Hex Utility** にチェック
4. **Output Format**: `Intel hex`

---

## モータードライバ通信

### コマンドフォーマット

```
$コマンド:パラメータ#
```

### 主要コマンド

| コマンド | 説明 | 例 |
|----------|------|-----|
| `$spd:a,b,c,d#` | 4輪の速度設定 (-255〜255 mm/s) | `$spd:100,100,100,100#` |
| `$pwm:a,b,c,d#` | 4輪のPWM直接設定 | `$pwm:50,50,50,50#` |

### エンコーダ値の取得

1. `send_upload_data(false, false, true)` で速度データ送信を有効化
2. `g_recv_flag` が 1 になったら `Deal_data_real()` を呼ぶ
3. `g_Speed[0]〜g_Speed[3]` に速度値（mm/s）が格納される

---

## IMU接続

### IMUモジュール（Yahboom 9軸IMU）
- 3.3V / 5V 両対応
- I2C / UART 両対応
- 製品ページ: https://www.yahboom.net/study/IMU_Sensor

### 接続先: ④番 IIC コネクタ（PA12/PA13）

| 拡張ボードコネクタ | ピン配置 | 用途 |
|-------------------|---------|------|
| ④ IIC | 5V, PA12(SCL), PA13(SDA), GND | IMU接続 |

### I2C通信仕様

- **I2Cアドレス**: `0x23`
- **I2Cクロック**: 100kHz
- **方式**: ソフトウェアビットバンギング（GPIO制御）
- **連続読み出し対応**: レジスタアドレス自動インクリメント
- **データ形式**: リトルエンディアン

### I2Cレジスタマップ

| レジスタ | 名前 | 型 | 説明 |
|----------|------|-----|------|
| 0x01-0x03 | VERSION | uint8 x3 | ファームウェアバージョン |
| 0x04-0x09 | ACCEL X/Y/Z | int16 x3 | 加速度（scale: 16/32767 g） |
| 0x0A-0x0F | GYRO X/Y/Z | int16 x3 | ジャイロ（scale: (2000/32767)*(pi/180) rad/s） |
| 0x10-0x15 | MAG X/Y/Z | int16 x3 | 磁気（scale: 800/32767） |
| 0x16-0x25 | QUAT W/X/Y/Z | float x4 | クォータニオン（IEEE754 LE） |
| 0x26-0x31 | EULER Roll/Pitch/Yaw | float x3 | オイラー角（IEEE754 LE、ラジアン） |
| 0x32-0x41 | BARO | float x4 | 気圧計（10軸のみ） |
| 0x61 | ALGO_TYPE | uint8 | アルゴリズム（6=6軸, 9=9軸） |

### オイラー角の読み方

```c
// レジスタ 0x26 から12バイト連続読み出し → 3つの IEEE754 float (LE, ラジアン)
i2cRead(0x23, 0x26, 12, buf);
// buf[0..3] = Roll, buf[4..7] = Pitch, buf[8..11] = Yaw
// 度数変換: degrees = radians * 57.2958
```

---

## Windows環境構築

### 必要なソフトウェア

| ソフトウェア | 用途 | ダウンロード |
|-------------|------|-------------|
| MSPM0 SDK | 開発用SDK | https://www.ti.com/tool/MSPM0-SDK |
| CCS Theia | IDE | https://www.ti.com/tool/CCSTUDIO |
| UniFlash | 書き込み | https://www.ti.com/tool/UNIFLASH |
| CH340ドライバ | USB-シリアル | https://www.wch.cn/download/CH341SER_EXE.html |

**重要**: すべてのインストールパスは **英語のみ**（日本語不可）。推奨: `C:\ti\`

### セットアップ手順

1. MSPM0 SDK を `C:\ti\` にインストール
2. CCS Theia を `C:\ti\` にインストール（コンポーネント: MSPM0 にチェック）
3. CCS で SDK パスを設定: `File` → `Preferences` → `Products`
4. UniFlash をインストール
5. CH340 ドライバを確認（デバイスマネージャーで COM ポート認識）

---

## ハードウェア構成（ボード上の部品）

### コアボード（MSPM0G3507 開発ボード）

| 部品 | ピン | 説明 |
|------|------|------|
| ユーザーLED D1 | PB2 | GPIO出力、ユーザー定義 |
| ユーザーLED D2 | PB3 | GPIO出力、ユーザー定義 |
| 電源LED | — | 電源インジケータ（制御不可） |
| リセットボタン | NRST | ハードウェアリセット |
| BSLボタン | — | ブートローダー起動用 |
| Type-C | — | USB（CH340 経由 UART0） |
| SWD | — | デバッグインターフェース（予約） |

### 拡張ボード（MSPM0 Robot Expansion Board）

拡張ボード自体にはLEDは搭載されていない。サンプルコードの LED 操作はコアボード上の D1/D2 を制御している。

| # | 部品 | 説明 |
|---|------|------|
| 1 | 4ch エンコーダモータードライバ I/F | UART1接続 |
| 2 | シリアルポート I/F (Uart1) | |
| 3 | シリアルポート I/F (Uart3) | |
| 4 | IIC I/F | IMU接続用（PA12/PA13） |
| 5 | IIC0 I/F | |
| 6 | IIC1 I/F | |
| 7 | カスタムボタン K1〜K4 | K1: GPIOA pin 2 (IOMUX_PINCM7)、K2〜K4は要確認 |
| 8 | シリアルポート (UART) | Bluetooth 5.0 等 |
| 9 | IIC I/F | OLED等 |
| 10 | 電源スイッチ | |
| 11 | 5-12V 電源入力 | |
| 12 | 電源出力 | |
| 13 | ブザー | GPIOB pin 24 (IOMUX_PINCM52)、TIMA0_CCP3 でPWM駆動 |
| 14 | 電磁パトロール I/F | |
| 15 | 4路ライントレース I/F | |
| 16 | 3路ライントレース I/F | |
| 17 | CCD モジュール I/F | |
| 18 | 40pin GPIO ピンヘッダ | |

### 活用案

- **LED D1 (PB2)**: IMUステータス表示（正常=点灯、異常=点滅）
- **LED D2 (PB3)**: モータードライバステータス表示
- **ボタン K1**: モーター非常停止（ハードウェア割り込み）
- **ブザー (PB24)**: 通電時ビープ音、エラー通知

### ピン競合に注意

- PB2/PB3 は現在のファームウェアでは未使用（追加可能）
- PB24（ブザー）は TIMA0_CCP3。現在 TIMA0 は 100ms タイマーとして使用中なので、ブザー用に別チャンネル設定するか TIMG を使う必要あり

---

## モーターパラメータ

### 520 L型モーター（現在使用中）

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| MOTOR_TYPE_ID | 1 | L型モーター |
| MOTOR_PULSE_PHASE | 40 | エンコーダパルス位相 |
| MOTOR_PULSE_LINE | 11 | エンコーダパルスライン |
| MOTOR_WHEEL_DIA | 67.00 mm | ホイール直径 |
| MOTOR_DEADZONE | 1900 | デッドゾーン |

### 差動駆動の符号規約

- M1（左輪）: コマンド正 = 前進
- M2（右輪）: コマンドを**符号反転**して送信（`Contrl_Speed(+left, -right, 0, 0)`）
- 速度・エンコーダフィードバックも M2 は符号反転して返す
- 停止時は `Contrl_Pwm(0,0,0,0)` を使用（`Contrl_Speed(0,...)` だとPIDが残り振動する）

---

## Modbus RTU 通信仕様

### 通信パラメータ

| 項目 | 値 |
|------|-----|
| プロトコル | Modbus RTU |
| 物理層 | UART_0 (PA25/PA26)、GPIO UART 直結（RPi GPIO14/15） |
| ボーレート | 115200 bps |
| データ | 8bit, パリティなし, 1ストップビット (8N1) |
| スレーブアドレス | 1 |
| フレーム区切り | 約2ms無通信（TIMER1 ワンショット検出） |

### 対応ファンクションコード

| FC | 名称 | 説明 |
|----|------|------|
| 0x03 | Read Holding Registers | レジスタ読み出し（最大125レジスタ） |
| 0x06 | Write Single Register | 単一レジスタ書き込み（0x40-0x41のみ） |
| 0x10 | Write Multiple Registers | 複数レジスタ書き込み（0x40-0x41のみ） |

それ以外のファンクションコードには例外応答 (ILLEGAL_FUNCTION) を返す。

### Modbus レジスタマップ（Holding Registers）

すべて 16bit。アドレスは 0-based。int16 の値は Modbus 標準に従いビッグエンディアンで送受信される。

#### ステータス (0x00–0x01) — 読み取り専用

| Addr | 名前 | 型 | 説明 |
|------|------|-----|------|
| 0x00 | STATUS | uint16 | bit0: IMU_OK, bit1: MOTOR_OK |
| 0x01 | HEARTBEAT | uint16 | 100ms毎にインクリメント（通信生存確認） |

#### IMU 生値 (0x10–0x18) — 読み取り専用

| Addr | 名前 | 型 | 単位変換 |
|------|------|-----|----------|
| 0x10 | ACCEL_X | int16 | raw × 16/32768 → g、× 9.80665 → m/s² |
| 0x11 | ACCEL_Y | int16 | 同上 |
| 0x12 | ACCEL_Z | int16 | 同上 |
| 0x13 | GYRO_X | int16 | raw × 2000/32768 → deg/s、× π/180 → rad/s |
| 0x14 | GYRO_Y | int16 | 同上 |
| 0x15 | GYRO_Z | int16 | 同上 |
| 0x16 | MAG_X | int16 | raw × 800/32768 → μT相当 |
| 0x17 | MAG_Y | int16 | 同上 |
| 0x18 | MAG_Z | int16 | 同上 |

#### クォータニオン (0x20–0x27) — 読み取り専用

各 float は 2レジスタ（lo_word, hi_word）に格納。LE ワードオーダー。

| Addr | 名前 | 型 | 説明 |
|------|------|-----|------|
| 0x20-0x21 | QUAT_W | float (2reg) | IMU内蔵フィルタ出力 |
| 0x22-0x23 | QUAT_X | float (2reg) | |
| 0x24-0x25 | QUAT_Y | float (2reg) | |
| 0x26-0x27 | QUAT_Z | float (2reg) | |

**float の復元方法 (Python)**:
```python
import struct
lo_word = registers[0]  # 0x20
hi_word = registers[1]  # 0x21
raw = lo_word | (hi_word << 16)
value = struct.unpack('<f', struct.pack('<I', raw))[0]
```

#### エンコーダ・速度 (0x30–0x35) — 読み取り専用

| Addr | 名前 | 型 | 説明 |
|------|------|-----|------|
| 0x30 | ENC_L_HI | int16 | 左エンコーダ累積 上位16bit |
| 0x31 | ENC_L_LO | uint16 | 左エンコーダ累積 下位16bit |
| 0x32 | ENC_R_HI | int16 | 右エンコーダ累積 上位16bit |
| 0x33 | ENC_R_LO | uint16 | 右エンコーダ累積 下位16bit |
| 0x34 | SPEED_L | int16 | 左速度フィードバック (mm/s) |
| 0x35 | SPEED_R | int16 | 右速度フィードバック (mm/s) |

**int32 エンコーダ値の復元方法 (Python)**:
```python
enc = (registers[0] << 16) | registers[1]  # HI, LO
if enc >= 0x80000000:
    enc -= 0x100000000
```

#### モーター速度指令 (0x40–0x41) — 読み書き可能

| Addr | 名前 | 型 | 説明 |
|------|------|-----|------|
| 0x40 | CMD_SPEED_L | int16 | 左速度指令 (mm/s, 正=前進) |
| 0x41 | CMD_SPEED_R | int16 | 右速度指令 (mm/s, 正=前進) |

**安全タイムアウト**: 0x40-0x41 への書き込みが 500ms 途絶するとモーター自動停止。
継続走行するには 500ms 以内に繰り返し書き込む必要がある。

### 例外応答コード

| コード | 名称 | 発生条件 |
|--------|------|----------|
| 0x01 | ILLEGAL_FUNCTION | 未対応ファンクションコード |
| 0x02 | ILLEGAL_ADDRESS | レジスタ範囲外 or 書き込み不可レジスタへの書き込み |
| 0x03 | ILLEGAL_VALUE | 不正なパラメータ（FC16 のバイト数不一致等） |

### CRC-16 計算

- 多項式: 0xA001 (Modbus 標準)
- 初期値: 0xFFFF
- ビット単位計算（テーブル不使用、メモリ節約）

---

## 通信性能（ストレステスト結果）

115200 baud、USB-シリアル (CH340) 経由で PC から pymodbus で計測。

| テスト内容 | レイテンシ (平均) | スループット | エラー率 |
|-----------|-----------------|-------------|---------|
| 1レジスタ読み出し | 7.7 ms | ~131 reads/sec | 0% |
| IMU 9レジスタ一括読み出し | 8.9 ms | ~113 reads/sec | 0% |
| 全レジスタ 66個一括読み出し | 19.7 ms | ~51 reads/sec | 0% |
| 書き込み + 全センサー読み出し1サイクル | 33.6 ms | ~30 cycles/sec | 0% |
| 5秒間連続ポーリング (9レジスタ) | — | 112 reads/sec | 0% |

### ROS2 ノード設計の目安

| 用途 | 推奨ポーリング周波数 | 方式 |
|------|---------------------|------|
| 制御ループ (cmd_vel → motor) | 30 Hz | FC16 書き込み + FC03 読み出し |
| IMU パブリッシュ | 50 Hz | 全レジスタ一括読み出し (0x00〜0x41) |
| オドメトリ | 50 Hz | IMUと同一サイクルで取得 |

**ボトルネック**: 115200 baud のシリアル転送速度。全66レジスタ一括で ~20ms なので、
書き込み + 全読み出しを1サイクルにまとめて **30〜50 Hz** が現実的な上限。

### Python テスト例 (pymodbus 3.x)

```python
from pymodbus.client import ModbusSerialClient
import struct

client = ModbusSerialClient(port='/dev/ttyUSB0', baudrate=115200, timeout=0.5)
client.connect()

# ステータス読み出し
result = client.read_holding_registers(0x00, count=2, device_id=1)
status = result.registers[0]
print(f"IMU_OK={bool(status & 1)}, Motor_OK={bool(status & 2)}")

# IMU + Mag 読み出し (0x10〜0x18, 9レジスタ)
result = client.read_holding_registers(0x10, count=9, device_id=1)
# int16 に変換
vals = [r if r < 0x8000 else r - 0x10000 for r in result.registers]

# Quaternion 読み出し (0x20〜0x27, 8レジスタ)
result = client.read_holding_registers(0x20, count=8, device_id=1)
regs = result.registers
for i in range(4):
    raw = regs[i*2] | (regs[i*2+1] << 16)
    f = struct.unpack('<f', struct.pack('<I', raw))[0]
    print(f"Q[{i}] = {f:.4f}")

# 速度指令 (左100mm/s, 右100mm/s = 直進)
client.write_registers(0x40, values=[100, 100], device_id=1)

# 停止
client.write_registers(0x40, values=[0, 0], device_id=1)

client.close()
```

**注意**: pymodbus 3.x では `slave=` ではなく `device_id=` を使用。
Linuxでは `/dev/ttyUSB0`、Windowsでは `COM3` 等を指定。

---

*最終更新: 2026-02-01*
