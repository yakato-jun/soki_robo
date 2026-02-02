# matplotlibで可視化

**目的**: 計算結果やセンサデータを目で見えるようにする。

---

## なぜ可視化が必要か

数字の羅列を眺めても、何が起きているか分かりません。

```
[1.2, 1.3, 1.1, 5.7, 1.2, 1.3, 1.0]
```

これだけ見て「4番目がおかしい」と気づくのは、データが少ないから。
100個、1000個になったら無理です。

グラフにすると一瞬で分かります。

> **見えれば分かる。見えなければ分からない。**

---

## matplotlibとは

Pythonでグラフを描くためのライブラリです。

```python
import matplotlib.pyplot as plt
```

これで `plt` として使えます。

---

## 基本的なグラフ

### 折れ線グラフ

データの変化を見るのに使います。

```python
import matplotlib.pyplot as plt
import numpy as np

time = np.arange(0, 5, 0.1)        # 0〜5秒、0.1秒刻み
speed = np.sin(time) * 0.5 + 0.5   # 適当な速度データ

plt.figure(figsize=(8, 4))
plt.plot(time, speed)
plt.xlabel("time [s]")
plt.ylabel("speed [m/s]")
plt.title("Speed over time")
plt.grid(True)
plt.show()
```

### 散布図

点の分布を見るのに使います。

```python
x = np.random.randn(100)
y = np.random.randn(100)

plt.figure(figsize=(6, 6))
plt.scatter(x, y, s=10)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Random points")
plt.axis('equal')
plt.grid(True)
plt.show()
```

---

## センサデータのプロット

### エンコーダの値を時系列で見る

```python
# エンコーダの値を記録したリストがあるとして
times = [0.0, 0.1, 0.2, 0.3, 0.4, ...]
encoder_left = [0, 12, 25, 37, 50, ...]
encoder_right = [0, 11, 24, 36, 49, ...]

plt.figure(figsize=(8, 4))
plt.plot(times, encoder_left, label="left")
plt.plot(times, encoder_right, label="right")
plt.xlabel("time [s]")
plt.ylabel("encoder count")
plt.title("Encoder values")
plt.legend()
plt.grid(True)
plt.show()
```

### IMUの加速度を見る

```python
# IMUの加速度データがあるとして
plt.figure(figsize=(8, 4))
plt.plot(times, accel_x, label="accel_x")
plt.plot(times, accel_y, label="accel_y")
plt.plot(times, accel_z, label="accel_z")
plt.xlabel("time [s]")
plt.ylabel("acceleration [m/s²]")
plt.title("IMU Acceleration")
plt.legend()
plt.grid(True)
plt.show()
```

---

## LiDAR点群の2D表示

06_lidarで先取りしたコードと同じですが、もう少し丁寧に:

```python
import numpy as np
import matplotlib.pyplot as plt

# LiDARスキャンデータ（角度[deg], 距離[mm]のリスト）
scan = robot.get_lidar_scan()

angles_deg = np.array([a for a, d in scan])
distances = np.array([d for a, d in scan])

# 極座標 → 直交座標
angles_rad = np.radians(angles_deg)
x = distances * np.cos(angles_rad)
y = distances * np.sin(angles_rad)

plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=1, c='blue')
plt.scatter(0, 0, s=50, c='red', marker='x', label='robot')  # ロボットの位置
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.title("LiDAR Scan")
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
```

ロボットを中心に、周囲の壁や障害物の形が見えます。

---

## 覚えておくパターン

```python
plt.figure(figsize=(幅, 高さ))    # 図のサイズ
plt.plot(x, y, label="ラベル")     # 折れ線
plt.scatter(x, y, s=点の大きさ)    # 散布図
plt.xlabel("X軸のラベル")
plt.ylabel("Y軸のラベル")
plt.title("タイトル")
plt.legend()                        # 凡例
plt.grid(True)                      # グリッド線
plt.axis('equal')                   # 縦横比を揃える
plt.show()                          # 表示
```

この基本パターンだけで、このレッスンで必要な可視化はほぼ全部できます。

---

## ここで覚えること

- matplotlib はデータをグラフにする道具
- 折れ線グラフ: 時系列データの変化を見る
- 散布図: 点の分布（LiDAR点群など）を見る
- 「見えれば分かる」を実践する道具

大枠2以降で計算結果を確認するとき、matplotlibで「本当にそうなっているか」を
目で見て確認する習慣をつけます。

---

**次のレッスン**: → #16 線形代数: 行列に戻る + 1文字抽象化 (`02_engineering_math/02_linear_algebra` の「行列に戻る」から再開)
