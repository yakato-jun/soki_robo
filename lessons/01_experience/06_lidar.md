# LiDARを読む

**目的**: 外界センサの体験。「周りに何があるかを知る」。

---

## LiDARとは

**Light Detection and Ranging**（光による検出と測距）。
光（レーザー）を使って、周囲の物体までの距離を測るセンサです。

### 動作原理（直感）

1. レーザーを一方向に発射する
2. 物体に当たって跳ね返ってくる
3. 往復時間から距離を計算する
4. これを360°ぐるっと繰り返す

結果として「周囲360°にある物体までの距離のリスト」が得られます。
これを **点群データ** と呼びます。

---

## LiDARはModbusではない

ここまでのセンサ（エンコーダ、IMU）は MCU 内蔵で、Modbus レジスタ経由で読みました。

LiDAR（RPLIDAR A1M8）は **独立したデバイス** です。
USB で直接ラズパイに繋がっており、専用のライブラリで通信します。

```
MCU (Modbus) ←USB→ ラズパイ ←USB→ LiDAR (独自プロトコル)
```

「すべてのセンサが同じ方法で読めるわけではない」— これも重要な学びです。

---

## 接続

```python
from rplidar import RPLidar

# LiDARに接続（ポートは環境による）
lidar = RPLidar(port='/dev/ttyUSB1', baudrate=115200, timeout=1)

# デバイス情報を確認
info = lidar.get_info()
print(info)
# {'model': 24, 'firmware': (1, 29), 'hardware': 7, 'serialnumber': '...'}

# 健康状態を確認
health = lidar.get_health()
print(health)
# ('Good', 0)
```

`rplidar-roboticia` というライブラリを使います（`pip install rplidar-roboticia`）。

**注意**: MCU（Modbus）と LiDAR は別の USB ポートに繋がっています。
`/dev/ttyUSB0` が MCU、`/dev/ttyUSB1` が LiDAR、という具合ですが、
接続順で入れ替わることがあります。

---

## 点群データとは

LiDARが返すデータは、こんな形です:

```python
# 1回転分のスキャン = (品質, 角度[度], 距離[mm]) のリスト
# [(quality, angle, distance), ...]
# 例: [(15, 0.5, 1200), (15, 1.4, 1210), (14, 2.3, 1195), ...]
```

各点は「この角度の方向に、この距離に何かがある」を意味します。
`quality` は反射光の強さで、0 は無効な測定です。

---

## スキャンデータを読む

```python
for i, scan in enumerate(lidar.iter_scans()):
    # scan = [(quality, angle, distance), ...]
    # 無効な測定（distance=0）を除外
    valid = [(q, a, d) for q, a, d in scan if d > 0]
    print(f"スキャン{i}: {len(valid)} 点")

    for q, a, d in valid[:5]:  # 最初の5点だけ表示
        print(f"  角度: {a:.1f}°, 距離: {d:.0f}mm, 品質: {q}")

    if i >= 2:
        break
```

`iter_scans()` は1回転ごとにデータをまとめて返すジェネレータです。
A1M8 は毎秒約5.5回転するので、1回転あたり約300〜400点が取れます。

### 終了処理は必ず行う

```python
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
```

**これを忘れるとモーターが回り続けます。** LiDAR のモーターは物理的に回転しているので、
プログラムが終了しても自動では止まりません。

---

## やってみよう

### 手を近づける・遠ざける

LiDARの前に手を出してみましょう。

```python
for i, scan in enumerate(lidar.iter_scans()):
    valid = [(q, a, d) for q, a, d in scan if d > 0]
    # 正面（0°付近）の距離を見る
    front = [(a, d) for _, a, d in valid if a < 10 or a > 350]
    for a, d in front:
        print(f"角度: {a:.1f}°, 距離: {d:.0f}mm")
    if i >= 5:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
```

- 手を近づけると距離が小さくなる
- 手を遠ざけると距離が大きくなる
- 手を動かすと値がリアルタイムに変わる

### 周囲の形を見る

```python
import numpy as np
import matplotlib.pyplot as plt

# 1スキャン分のデータを取得
for scan in lidar.iter_scans():
    valid = [(a, d) for _, a, d in scan if d > 0]
    break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

angles = np.array([a for a, d in valid]) * np.pi / 180  # deg→rad
distances = np.array([d for a, d in valid])

# 極座標→直交座標に変換（三角関数の伏線！）
x = distances * np.cos(angles)
y = distances * np.sin(angles)

# matplotlibで描画（08_matplotlibの先取り）
plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=1)
plt.axis('equal')
plt.title('LiDAR scan')
plt.show()
```

壁や机や人の形が見えるはずです。

### ロボットを回して見る

ロボットをその場で回しながらLiDARデータを見ます。

**気づくこと:**
- ロボットが回ると、データの向きも回る
- 同じ壁なのに、ロボットの向きによって違う角度に見える
- **LiDARが見ているのは「ロボットから見た世界」であって「世界そのもの」ではない**

---

## エンコーダ・IMU・LiDARの比較

| センサ | 分類 | 何が分かる | 通信方式 | 弱点 |
| ------ | ---- | ---------- | -------- | ---- |
| エンコーダ | 内界 | どれだけ動いたか | Modbus (MCU内蔵) | 誤差が累積する |
| IMU | 内界 | どんな姿勢・動きか | Modbus (MCU内蔵) | ノイズ、ドリフト |
| LiDAR | 外界 | 周りに何があるか | 独自プロトコル (USB) | ロボット自身の位置は分からない |

1つのセンサでは情報が足りません。
だから複数のセンサを **組み合わせて** 使います。

この「組み合わせ方」が大枠3で学ぶ **センサフュージョン** であり、
その土台になるのが大枠2の **確率** です。

---

## 気づいてほしいこと

- LiDARは「周囲の形」を点群として返す外界センサ
- 角度と距離のペアで表現される（極座標）
- ロボットを回すとデータの向きも変わる（座標系の問題、再び）
- x, yに変換するのに `cos` と `sin` を使った → **三角関数の伏線**
- 1つのセンサだけでは不十分。組み合わせが重要
- **すべてのセンサが同じ方法で読めるわけではない**（Modbus vs 専用ライブラリ）
- 終了処理を忘れるとハードウェアが止まらない → **リソース管理の意識**

---

**次のレッスン**: → #15 matplotlibで可視化 (`01_experience/08_matplotlib`)
