# numpy入門

**目的**: 数学の計算道具を手に入れる。「手計算めんどくさいでしょ → 1行で終わる」。

---

## numpyとは

**Numerical Python** の略。数値計算のためのライブラリです。

ベクトルや行列の計算が、たった1行で書けます。
大枠2（工学的数学）で出てくる計算は、ほぼすべてnumpyで実行します。

```python
import numpy as np
```

これで `np` として使えるようになります。

---

## ベクトル — 数字の束

ベクトルは「意味を持った数字の束」です。

```python
# 位置ベクトル: x=3, y=4
position = np.array([3, 4])

# 速度ベクトル: vx=1, vy=2
velocity = np.array([1, 2])
```

### 基本操作

```python
a = np.array([3, 4])
b = np.array([1, 2])

print(a + b)     # [4, 6]   ← 足し算（成分ごと）
print(a - b)     # [2, 2]   ← 引き算（成分ごと）
print(a * 2)     # [6, 8]   ← スカラー倍
print(a * b)     # [3, 8]   ← 成分ごとの掛け算（注意: 行列積ではない）
```

---

## 行列 — 変換ルールの表

行列は「入力を出力に変換するルール」です。

```python
# 2×2の行列
A = np.array([[1, 2],
              [3, 4]])

print(A)
# [[1 2]
#  [3 4]]
```

---

## 行列積 — `@` 演算子

行列の掛け算は `@` を使います。

```python
A = np.array([[1, 2],
              [3, 4]])

v = np.array([5, 6])

result = A @ v
print(result)  # [17, 39]
```

手で計算すると:
- 1×5 + 2×6 = 17
- 3×5 + 4×6 = 39

手計算は面倒ですが、numpyなら `A @ v` の1行です。

### 行列同士の積

```python
A = np.array([[1, 2],
              [3, 4]])

B = np.array([[5, 6],
              [7, 8]])

print(A @ B)
# [[19 22]
#  [43 50]]
```

---

## よく使う関数

### 内積: `np.dot`

```python
a = np.array([1, 0])
b = np.array([0, 1])
print(np.dot(a, b))  # 0  ← 直交していると0
```

### ノルム（長さ）: `np.linalg.norm`

```python
v = np.array([3, 4])
print(np.linalg.norm(v))  # 5.0  ← 三平方の定理: √(9+16)=5
```

### 正規化（方向だけ取り出す）

```python
v = np.array([3, 4])
unit_v = v / np.linalg.norm(v)
print(unit_v)  # [0.6, 0.8]  ← 長さ1のベクトル
```

### 外積: `np.cross`

```python
a = np.array([1, 0, 0])
b = np.array([0, 1, 0])
print(np.cross(a, b))  # [0, 0, 1]  ← 両方に垂直な方向
```

### 三角関数

```python
angle_rad = np.radians(45)       # 45° → rad
print(np.sin(angle_rad))         # 0.7071...
print(np.cos(angle_rad))         # 0.7071...
print(np.degrees(np.pi / 4))     # 45.0  ← rad → °
```

### 逆三角関数

```python
print(np.arccos(0))              # 1.5707... ← π/2
print(np.degrees(np.arctan2(1, 1)))  # 45.0  ← atan2は象限を考慮
```

---

## まとめ

| やりたいこと | numpy | 手計算 |
| ------------ | ----- | ------ |
| ベクトルを作る | `np.array([x, y])` | — |
| 行列を作る | `np.array([[a,b],[c,d]])` | — |
| 行列積 | `A @ B` | 展開して足し算 |
| 内積 | `np.dot(a, b)` | 成分ごとに掛けて足す |
| ノルム | `np.linalg.norm(v)` | √(x²+y²) |
| 正規化 | `v / np.linalg.norm(v)` | 各成分÷長さ |
| 外積 | `np.cross(a, b)` | 公式に当てはめる |
| sin/cos | `np.sin(r)` / `np.cos(r)` | 表を引く |
| deg↔rad | `np.radians()` / `np.degrees()` | ×π/180 |

道具は揃いました。あとは「何を計算するか」だけです。
それが大枠2（工学的数学）と大枠3（工学演算）で学ぶ内容です。

---

**次のレッスン**: → #10 線形代数 (`02_engineering_math/02_linear_algebra`)
