# エンコーダを読む

**目的**: 内界センサの体験。「自分がどれだけ動いたかを知る」。

---

## エンコーダとは

車輪の回転量を数えるセンサです。

車輪が回ると、エンコーダの値が増えていきます。
逆に回すと、値が減ります。

これだけです。やっていることは非常にシンプルです。

### なぜ必要か

前のレッスンで `set_speed(100, 100)` と書きましたが、
実は **本当にその速度で回っているかは分かりません**。

- 床が滑るかもしれない
- バッテリーの電圧が変わるかもしれない
- 負荷がかかっているかもしれない

「命令した」と「実際にそう動いた」は別物です。
エンコーダは「実際にどれだけ動いたか」を教えてくれるセンサです。

---

## 速度指令の裏側 — PIDとPWM

### モーターを回す2つの方法

モーターを回す方法は2種類あります。

**PWM（パルス幅変調）**: モーターへの電力を直接制御する。
- 「電力の何%を送るか」を指定する
- 0% = 停止、100% = 全力
- 床が重くても電力は同じ → **速度は保証されない**

**速度指令 (mm/s)**: 目標速度を指定する。
- 「100mm/sで走れ」と命令する
- ファームウェアが内部でエンコーダを読み、目標との差を計算し、PWMを自動調整する
- これが **PID制御**（大枠3で詳しく学びます）

前のレッスンで `set_speed(100, 100)` と書いたとき、裏ではこうなっています:

```
速度指令 100mm/s → ファーム内部でエンコーダ読み取り
                  → 実際の速度と比較
                  → PWMを自動調整（PID）
                  → モーターが回る
```

つまり、エンコーダはロボット内部でも常に使われています。
ここで学ぶのは、**自分のプログラムからも読む** 方法です。

---

## Modbusでエンコーダを読む

### レジスタマップ

| アドレス | 名前 | 型 | 説明 |
|----------|------|-----|------|
| 0x30 | ENC_L_HI | int16 | 左エンコーダ累積 上位16bit |
| 0x31 | ENC_L_LO | uint16 | 左エンコーダ累積 下位16bit |
| 0x32 | ENC_R_HI | int16 | 右エンコーダ累積 上位16bit |
| 0x33 | ENC_R_LO | uint16 | 右エンコーダ累積 下位16bit |
| 0x34 | SPEED_L | int16 | 左速度フィードバック (mm/s) |
| 0x35 | SPEED_R | int16 | 右速度フィードバック (mm/s) |

### 速度フィードバックを読む

速度フィードバック（0x34, 0x35）は int16 です。車輪制御と同じ符号変換が要ります。

```python
result = client.read_holding_registers(0x34, count=2, device_id=1)
regs = result.registers

def to_int16(val):
    """Modbus の uint16 を int16 に変換"""
    return val if val < 0x8000 else val - 0x10000

speed_l = to_int16(regs[0])  # 左速度 [mm/s]
speed_r = to_int16(regs[1])  # 右速度 [mm/s]
print(f"左: {speed_l} mm/s, 右: {speed_r} mm/s")
```

車輪制御では `to_uint16`（Python → Modbus）を作りました。
今度は `to_int16`（Modbus → Python）です。**行きと帰りで変換が逆。**

### エンコーダ累積値を読む — 32bitの壁

エンコーダの累積値は車輪が回り続けると大きくなるため、16bitでは足りません。
そこで **32bit（int32）を2つのレジスタに分割** して格納しています。

```python
result = client.read_holding_registers(0x30, count=4, device_id=1)
regs = result.registers

def to_int32(hi, lo):
    """2つの16bitレジスタから int32 を復元"""
    val = (hi << 16) | lo
    if val >= 0x80000000:
        val -= 0x100000000
    return val

enc_l = to_int32(regs[0], regs[1])  # 左エンコーダ累積
enc_r = to_int32(regs[2], regs[3])  # 右エンコーダ累積
print(f"左: {enc_l}, 右: {enc_r}")
```

`to_int16` に続いて `to_int32` の変換関数を作りました。
パターンは同じ: **レジスタの生値 → 意味のある数値に変換。**

---

## 便利関数にまとめる

```python
def read_speed():
    """速度フィードバックを読む [mm/s]"""
    result = client.read_holding_registers(0x34, count=2, device_id=1)
    regs = result.registers
    return to_int16(regs[0]), to_int16(regs[1])

def read_encoder():
    """エンコーダ累積値を読む"""
    result = client.read_holding_registers(0x30, count=4, device_id=1)
    regs = result.registers
    return to_int32(regs[0], regs[1]), to_int32(regs[2], regs[3])
```

---

## やってみよう

### 車輪を手で回す

ロボットの電源を入れた状態で、車輪を手で回してみましょう。

```python
for i in range(10):
    enc_l, enc_r = read_encoder()
    print(f"左: {enc_l}, 右: {enc_r}")
    time.sleep(0.5)
```

- 前に回すと値が増える
- 後ろに回すと値が減る
- 速く回すと値の変化が大きい

### 走らせて読む

速度指令を出しながら、フィードバックを読みます。

```python
for i in range(20):
    set_speed(100, 100)
    speed_l, speed_r = read_speed()
    enc_l, enc_r = read_encoder()
    print(f"速度 L:{speed_l:>4d} R:{speed_r:>4d} mm/s | 累積 L:{enc_l} R:{enc_r}")
    time.sleep(0.1)
stop()
```

速度フィードバックは **命令値（100mm/s）とぴったり同じにはならない** はずです。
ファーム内部のPIDが追従しようとしていますが、常に少しズレています。

### 命令と実際の差を見る

```python
target = 100  # mm/s
for i in range(30):
    set_speed(target, target)
    speed_l, speed_r = read_speed()
    error_l = target - speed_l
    error_r = target - speed_r
    print(f"誤差 L:{error_l:>4d} R:{error_r:>4d} mm/s")
    time.sleep(0.1)
stop()
```

命令と実際の差 = **誤差**。PID制御はこの誤差をゼロに近づけようとする仕組みです。

---

## 回転量と移動距離の関係

エンコーダの値は「パルス数」です。
移動距離に変換するには車輪の大きさが要ります。

このロボットのパラメータ:
- ホイール直径: 67.0 mm
- エンコーダ: 11ライン × 40パルス/相

直感レベルで:
- 車輪1回転 = エンコーダが一定量増える
- 車輪の円周 = 直径 × π = 1回転あたりの移動距離
- エンコーダの変化量 → 車輪の回転量 → 移動距離

具体的な変換式は大枠3（ホイールオドメトリ）で扱います。
ここでは「エンコーダを読めば、どれだけ動いたか分かる」が伝わればOKです。

---

## 気づいてほしいこと

- エンコーダは「過去にどれだけ動いたか」の累積値
- 速度フィードバックを見ると、命令値とは常に少しズレている
- 32bitの値を16bit×2に分割して送る → **レジスタの制約に合わせたデータ設計**
- `to_int16`, `to_int32` — 変換関数が増えてきた。**パターンは同じ**
- 同じ速度命令でも、左右のエンコーダ値が完全に同じにはならない
  → **現実のセンサには常に誤差がある**

この「誤差が積み上がっていく」問題は、大枠3のホイールオドメトリで
実際に体で感じることになります。

---

**次のレッスン**: → #08 IMUを読む (`01_experience/05_imu`)
