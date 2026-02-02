# ホイールオドメトリ

**前提知識**: 微積（02/03）、エンコーダ（01/04）

**目的**: エンコーダの値から位置・姿勢を推定する。離散積分の実践。

---

## オドメトリとは

「自分がどれだけ動いたか」を内界センサ（エンコーダ）だけで推定すること。

エンコーダの値を微積で処理して、ロボットの位置と姿勢を求めます。

---

## 基本の流れ

```
エンコーダ値 → 車輪速度 → 機体速度 → 位置・姿勢の更新
```

1. **エンコーダの差分** → 車輪の回転量
2. **車輪の回転量 / dt** → 車輪速度 `[Vl, Vr]`（離散微分）
3. **ξ = J W** → 機体速度 `[vx, ωz]`（差動二輪の運動学）
4. **位置の更新** → `x += vx * cos(θ) * dt`, `y += vx * sin(θ) * dt`, `θ += ωz * dt`（離散積分）

大枠2で学んだ道具がすべて合流します:
- 微積（差分と積分）
- 三角関数（cos, sin）
- 線形代数（J行列）

---

## numpyで実装

```python
import numpy as np

# ロボットのパラメータ
L = 0.15              # トレッド幅 [m]
wheel_radius = 0.033  # 車輪半径 [m]
dt = 0.1              # サンプリング周期 [s]

# 構造行列
J = np.array([[0.5,    0.5],
              [-1/L,   1/L]])

# 初期状態
x, y, theta = 0.0, 0.0, 0.0

# エンコーダの記録（仮のデータ）
encoder_left  = [0, 50, 100, 150, 200, 250, 300, 350]
encoder_right = [0, 50, 100, 150, 200, 250, 300, 350]

# 軌跡を記録
trajectory = [(x, y)]

for k in range(1, len(encoder_left)):
    # 1. エンコーダ差分 → 車輪速度
    dl = (encoder_left[k] - encoder_left[k-1]) * wheel_radius  # [m]
    dr = (encoder_right[k] - encoder_right[k-1]) * wheel_radius
    Vl = dl / dt
    Vr = dr / dt

    # 2. 構造行列で機体速度に変換
    W = np.array([Vl, Vr])
    xi = J @ W  # [vx, ωz]
    vx, wz = xi

    # 3. 位置・姿勢を更新（離散積分）
    x += vx * np.cos(theta) * dt
    y += vx * np.sin(theta) * dt
    theta += wz * dt

    trajectory.append((x, y))

# 結果を表示
for i, (px, py) in enumerate(trajectory):
    print(f"step {i}: x={px:.3f}, y={py:.3f}")
```

---

## 実際に走らせて軌跡を描く

```python
import matplotlib.pyplot as plt

traj = np.array(trajectory)
plt.figure(figsize=(8, 8))
plt.plot(traj[:, 0], traj[:, 1], 'b-o', markersize=3)
plt.plot(traj[0, 0], traj[0, 1], 'go', markersize=10, label='start')
plt.plot(traj[-1, 0], traj[-1, 1], 'ro', markersize=10, label='end')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Wheel Odometry Trajectory')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()
```

---

## 破綻体験: 累積誤差

**ここが最も重要な体験です。**

### やること

オドメトリだけで四角形を描かせます。

```python
# 1. 1m直進
# 2. 90°右旋回
# 3. 1m直進
# 4. 90°右旋回
# ... を4回繰り返す → 元の位置に戻る...はず
```

### 起きること

- 最初の1辺はほぼ正確
- 2辺目で少しズレる
- 3辺目、4辺目と走るほどズレが大きくなる
- **元の位置に戻ってこない**

### なぜ破綻するか

- 車輪が微妙に滑る（スリップ）
- 車輪径の個体差
- 床の凹凸
- エンコーダの分解能の限界

すべての誤差が **積分で累積** されていきます。
離散積分 `x += vx * dt` は、毎ステップの微小な誤差を足し続ける操作だからです。

### この体験が教えてくれること

> **理論通りに作っても、現実では必ず破綻する。**

これは設計ミスではなく、内界センサだけに頼る限り避けられない問題です。

だから:
- 外界センサ（LiDAR等）と組み合わせる → **センサフュージョン**
- 確率的に推定を補正する → **カルマンフィルタ**

が必要になります。

大枠2で学んだ確率と、この破綻体験が、
大枠3後半のフィルタ（相補フィルタ、カルマンフィルタ）を学ぶ最大の動機になります。

---

**次のレッスン**: → #24 PID制御 (`03_engineering_computation/03_pid_control`)
