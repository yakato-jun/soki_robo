# PID制御

**前提知識**: 微積（02/03）

**目的**: 微積の実践。P, I, Dの意味を理解し、ロボットの速度制御を体験する。

---

## PIDとは

**目標値に追従させる** ための制御手法です。

「ロボットを0.5 m/sで走らせたい」— でも現実では:
- 床の抵抗で遅くなる
- バッテリー電圧が変わる
- 坂道がある

「命令した速度」と「実際の速度」にはズレ（**誤差**）が出ます。
PIDはこの誤差を見ながら、命令を自動的に修正します。

---

## P, I, D の意味

```
制御量 = Kp × e(t) + Ki × ∫e(t)dt + Kd × de(t)/dt
```

離散版:

```
u[k] = Kp * e[k] + Ki * Σe * dt + Kd * (e[k] - e[k-1]) / dt
```

### P（Proportional）— 今の誤差に比例

```
P = Kp × e[k]
```

- 誤差が大きい → 大きく修正
- 誤差が小さい → 小さく修正
- 直感的で分かりやすい

**問題**: Pだけだと、誤差がゼロにならないことがある（定常偏差）

### I（Integral）— 過去の誤差の累積（離散積分）

```
I = Ki × Σ(e[k] * dt)
```

- 小さい誤差でも、溜まっていけば大きくなる
- 定常偏差を消す力がある

大枠2の **離散積分** そのものです。 `Σ(値 × dt)` = 面積の足し算。

**問題**: 溜まりすぎると暴走する（ワインドアップ）

### D（Derivative）— 誤差の変化率（離散微分）

```
D = Kd × (e[k] - e[k-1]) / dt
```

- 誤差が急に変わった → 大きくブレーキ
- 誤差が安定している → 何もしない
- 応答を速くする

大枠2の **離散微分** そのものです。 `差分 / dt` = 変化率。

**問題**: ノイズに弱い（ノイズ = 急な変化 → 過剰反応）

---

## numpyで実装

```python
import numpy as np

# PIDゲイン
Kp = 1.0
Ki = 0.1
Kd = 0.05

# シミュレーション設定
dt = 0.01   # 100Hz
target = 0.5  # 目標速度 [m/s]
actual = 0.0  # 現在速度
integral = 0.0
prev_error = 0.0

# 記録用
history_target = []
history_actual = []

for step in range(500):
    # 誤差
    error = target - actual

    # P
    P = Kp * error

    # I（離散積分）
    integral += error * dt
    I = Ki * integral

    # D（離散微分）
    derivative = (error - prev_error) / dt
    D = Kd * derivative
    prev_error = error

    # 制御量
    u = P + I + D

    # 簡易的なモーターモデル（1次遅れ）
    actual += (u - actual) * dt * 10

    history_target.append(target)
    history_actual.append(actual)

# プロット
import matplotlib.pyplot as plt
time = np.arange(len(history_actual)) * dt
plt.figure(figsize=(8, 4))
plt.plot(time, history_target, 'r--', label='target')
plt.plot(time, history_actual, 'b-', label='actual')
plt.xlabel('time [s]')
plt.ylabel('speed [m/s]')
plt.title('PID Speed Control')
plt.legend()
plt.grid(True)
plt.show()
```

---

## ゲインを変えてみよう

| パラメータ | 効果 | 大きすぎると |
| ---------- | ---- | ------------ |
| Kp | 誤差への反応速度 | 振動（オーバーシュート） |
| Ki | 定常偏差の除去 | ワインドアップ（暴走） |
| Kd | 振動の抑制 | ノイズに過敏 |

実際にゲインを変えて、挙動がどう変わるか体験してみましょう。

---

## 破綻体験: PIDが暴れる

### Dゲインを大きくする

```python
Kd = 5.0  # 大きくする
```

→ ノイズで振動する。
Dは「変化率」を見ているので、ノイズ（急な変化）に過剰反応します。

### Iゲインを大きくする

```python
Ki = 10.0  # 大きくする
```

→ ワインドアップ（巻き上がり）。
誤差の累積が巨大になり、オーバーシュートが止まらなくなります。

### この体験が教えてくれること

> **理論的に正しいはずのPIDが、パラメータ次第で暴れる。**

制御は理論だけでは足りません。実機で調整する世界があります。

- ノイズにはフィルタが必要 → 確率的手法の動機
- ワインドアップには対策が必要 → 実装上の工夫
- ゲインの調整 = 経験と試行錯誤の世界

これが制御工学の「実践」の部分で、大学の授業と実務の橋渡しになります。

---

**次のレッスン**: → #25 確率: ガチャとの違い (`02_engineering_math/04_probability`)
