# IMUを読む

**目的**: 内界センサの体験。「自分がどんな姿勢かを知る」。

---

## IMUとは

**Inertial Measurement Unit**（慣性計測装置）。
加速度・角速度・地磁気を測るセンサです。

### 9軸の意味

「9軸」と聞くと複雑に聞こえますが、3種類 × 3方向です。

| センサ | 何を測る | 3軸の意味 |
| ------ | -------- | --------- |
| 加速度センサ | どの方向にどれだけ力がかかっているか | x, y, z方向の加速度 |
| ジャイロセンサ | どの軸周りにどれだけ回っているか | x, y, z軸周りの角速度 |
| 地磁気センサ | どの方向に磁場があるか（≒方位） | x, y, z方向の地磁気 |

- 加速度 → 「どっちに動いているか」
- 角速度 → 「どれだけ回っているか」
- 地磁気 → 「どっちが北か」

3つ合わせると「今どんな姿勢で、どっちを向いていて、どう動いているか」が分かります。

---

## ModbusでIMUを読む

### レジスタマップ

| アドレス | 名前 | 型 | 内容 |
|----------|------|-----|------|
| 0x10 | ACCEL_X | int16 | 加速度X |
| 0x11 | ACCEL_Y | int16 | 加速度Y |
| 0x12 | ACCEL_Z | int16 | 加速度Z |
| 0x13 | GYRO_X | int16 | 角速度X |
| 0x14 | GYRO_Y | int16 | 角速度Y |
| 0x15 | GYRO_Z | int16 | 角速度Z |
| 0x16 | MAG_X | int16 | 地磁気X |
| 0x17 | MAG_Y | int16 | 地磁気Y |
| 0x18 | MAG_Z | int16 | 地磁気Z |

### 生値を読む

IMUの9軸データは連続する9レジスタ（0x10〜0x18）に並んでいます。
すべて int16 なので、エンコーダで作った `to_int16` がそのまま使えます。

```python
result = client.read_holding_registers(0x10, count=9, device_id=1)
regs = result.registers

# 全レジスタを int16 に変換
vals = [to_int16(r) for r in regs]

accel = vals[0:3]  # 加速度 [raw]
gyro  = vals[3:6]  # 角速度 [raw]
mag   = vals[6:9]  # 地磁気 [raw]

print(f"加速度: {accel}")
print(f"角速度: {gyro}")
print(f"地磁気: {mag}")
```

前のレッスンで作った `to_int16` がここでも使えます。
**同じ変換関数を再利用** — 関数にまとめておいてよかったですよね。

---

## 生値と物理量 — LSBスケーリング

### なぜ変換が必要か

上で読んだ値は「生値（raw value）」です。単位がありません。

- `accel_z = 2048` → これは何 m/s²？
- `gyro_z = -100` → これは何 rad/s？

センサはAD変換器（アナログ→デジタル変換）で値を取るので、
結果は整数になります。この整数を物理量に変換する係数が
**LSB（Least Significant Bit）スケーリング** です。

### このIMUのスケーリング

| センサ | 生値の範囲 | 物理量の範囲 | 変換式 |
|--------|-----------|-------------|--------|
| 加速度 | ±32768 | ±16 g | raw × 16/32768 × 9.80665 → m/s² |
| ジャイロ | ±32768 | ±2000 deg/s | raw × 2000/32768 × π/180 → rad/s |
| 地磁気 | ±32768 | ±800 μT相当 | raw × 800/32768 → μT |

```python
import math

# LSBスケーリング定数
ACCEL_SCALE = 16 / 32768 * 9.80665          # → m/s²
GYRO_SCALE  = 2000 / 32768 * math.pi / 180  # → rad/s
MAG_SCALE   = 800 / 32768                    # → μT相当

# 生値 → 物理量
accel_ms2 = [v * ACCEL_SCALE for v in accel]
gyro_rads = [v * GYRO_SCALE for v in gyro]
mag_ut    = [v * MAG_SCALE for v in mag]

print(f"加速度: x={accel_ms2[0]:.2f}, y={accel_ms2[1]:.2f}, z={accel_ms2[2]:.2f} m/s²")
print(f"角速度: x={gyro_rads[0]:.4f}, y={gyro_rads[1]:.4f}, z={gyro_rads[2]:.4f} rad/s")
print(f"地磁気: x={mag_ut[0]:.1f}, y={mag_ut[1]:.1f}, z={mag_ut[2]:.1f} μT")
```

`to_uint16`, `to_int16`, `to_int32` に続いて、今度は **LSBスケーリング**。
レジスタの生値 → 型変換 → 物理量変換。パターンは同じです。

---

## クォータニオンを読む — floatの壁

### Modbusでfloatを送る問題

クォータニオン（四元数）は4つの小数値（w, x, y, z）です。
しかし Modbus のレジスタは 16bit 整数 — **小数は直接送れません。**

そこで float（32bit浮動小数点）を **2つの16bitレジスタに分割** して格納しています。
エンコーダの int32 が2レジスタに分割されていたのと同じ考え方です。

### レジスタマップ

| アドレス | 名前 | 型 | 説明 |
|----------|------|-----|------|
| 0x20-0x21 | QUAT_W | float (2reg) | クォータニオン W |
| 0x22-0x23 | QUAT_X | float (2reg) | クォータニオン X |
| 0x24-0x25 | QUAT_Y | float (2reg) | クォータニオン Y |
| 0x26-0x27 | QUAT_Z | float (2reg) | クォータニオン Z |

### floatの復元

int32 は「ビットをつなげて符号変換」でした。
float は「ビットをつなげて **IEEE754 として解釈**」します。

```python
import struct

def to_float_le(lo, hi):
    """2つの16bitレジスタから float を復元（LEワードオーダー）"""
    raw = lo | (hi << 16)
    return struct.unpack('<f', struct.pack('<I', raw))[0]
```

`struct` モジュールは、バイト列と数値を相互変換する道具です。
- `struct.pack('<I', raw)` — 整数を4バイトのバイト列に変換
- `struct.unpack('<f', ...)` — そのバイト列を float として解釈
- `<` はリトルエンディアン（このIMUのデータ配置）

```python
result = client.read_holding_registers(0x20, count=8, device_id=1)
regs = result.registers

quat_w = to_float_le(regs[0], regs[1])
quat_x = to_float_le(regs[2], regs[3])
quat_y = to_float_le(regs[4], regs[5])
quat_z = to_float_le(regs[6], regs[7])

print(f"w={quat_w:.4f}, x={quat_x:.4f}, y={quat_y:.4f}, z={quat_z:.4f}")
```

`to_int16`, `to_int32`, そして `to_float_le`。
**16bitレジスタから元の型を復元する** — 道具が揃ってきました。

---

## 便利関数にまとめる

```python
def read_imu():
    """IMU 9軸の生値を物理量で返す"""
    result = client.read_holding_registers(0x10, count=9, device_id=1)
    vals = [to_int16(r) for r in result.registers]
    accel = [v * ACCEL_SCALE for v in vals[0:3]]  # m/s²
    gyro  = [v * GYRO_SCALE for v in vals[3:6]]   # rad/s
    mag   = [v * MAG_SCALE for v in vals[6:9]]     # μT
    return accel, gyro, mag

def read_quaternion():
    """クォータニオンを読む [w, x, y, z]"""
    result = client.read_holding_registers(0x20, count=8, device_id=1)
    regs = result.registers
    return [to_float_le(regs[i*2], regs[i*2+1]) for i in range(4)]
```

---

## やってみよう

### 静止状態で読む

ロボットを机の上に置いて、動かさずに値を読んでみましょう。

```python
for i in range(10):
    accel, gyro, mag = read_imu()
    print(f"accel_z={accel[2]:.2f} m/s², gyro_z={gyro[2]:.4f} rad/s")
    time.sleep(0.5)
```

**気づくこと:**
- 静止しているのに accel_z が約 9.8 → **重力**を測っている
- 角速度は 0 に近いが、完全な 0 ではない → **ノイズ**がある

### ロボットを手で傾ける

ロボットを手で持って、ゆっくり傾けてみましょう。

- 前に傾ける → accel_x が変わる
- 横に傾ける → accel_y が変わる
- 回す → gyro_z が変わる

### クォータニオンを見る

```python
for i in range(10):
    q = read_quaternion()
    print(f"w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}")
    time.sleep(0.5)
```

クォータニオンはIMU内蔵のフィルタが加速度・角速度・地磁気を融合して
算出した姿勢です。四元数の意味は大枠2の線形代数で触れます。
今は「4つの数字で姿勢を表現している」が伝わればOKです。

### ロボットを手で回す

ロボットを机の上でゆっくり回してみましょう。

**気づくこと:**
- 角速度zは回している間だけ値が出る（回転の速さ）
- 地磁気は向きによって値が変わる（方位）
- クォータニオンも向きに応じて値が変わる
- **IMUが示す「前」とロボットの「前」が一致しない場合がある**

---

## 座標のズレを体感する

最後の「IMUの前とロボットの前が一致しない」は重要です。

IMUセンサはロボットの基板に載っていますが、
基板の向きとロボットの「前」は必ずしも同じではありません。

- IMUは「センサ座標系」で値を返す
- ロボットの「前」は「ロボット座標系」

**この2つの座標系がズレている場合、IMUの値をそのまま使うと間違う。**

これが01_roadmapで話した「座標系」の問題です。
解決する道具（座標変換）は大枠2の線形代数で出てきます。

---

## 気づいてほしいこと

- IMUは「自分の動き」を測る内界センサ
- 静止していてもノイズがある（完璧なセンサは存在しない）
- 静止していても重力が測れる（これが姿勢推定に使える）
- センサの向きとロボットの向きは必ずしも一致しない（座標系の問題）
- 角速度を累積すれば向きが分かるが、ノイズも累積する（ドリフト問題）
- 生値 → 物理量の変換（LSBスケーリング）はセンサを使うたびに必要
- `to_int16`, `to_int32`, `to_float_le` — **16bitレジスタから元の型を復元するパターン**

> **補足**: クォータニオンはIMU内蔵フィルタが算出した姿勢出力です。
> 四元数の意味は大枠2の線形代数で「へー」レベルで触れます。
> 今は「floatの復元方法を学んだ」で十分です。

---

**次のレッスン**: → #09 numpy入門 (`01_experience/07_numpy`)
