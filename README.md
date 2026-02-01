# souki_robo

自律走行ロボットプロジェクト。

## ディレクトリ構成

```
souki_robo/
├── lessons/          # レッスン（段階的に学習）
├── python/           # Python サンプル・ライブラリ
│   ├── examples/     # サンプルスクリプト
│   └── requirements.txt
├── ros2_ws/          # ROS2 ワークスペース
│   └── src/
│       ├── souki_bringup/      # 起動・設定
│       ├── souki_description/  # URDF・モデル
│       └── souki_navigation/   # ナビゲーション
├── utils/            # ユーティリティ
│   └── service_manager/  # RPi サービス自動起動管理
└── firmware/         # MSPM0 ファームウェア
    ├── souki_main/   # メインファームウェア（Modbus RTU スレーブ）
    ├── examples/     # 個別機能の実験用プロジェクト
    └── knowhow.md    # ファームウェア開発ノウハウ
```

## ハードウェア構成

- **シャーシ**: Yahboom MSPM0 シャーシキット（差動二輪）
- **メインボード**: Raspberry Pi（高レベル制御）
- **マイコン**: MSPM0G3507（リアルタイム制御）
- **センサー**: エンコーダ x2、9軸IMU、RP LiDAR A1M8

## システム構成

```
Raspberry Pi (Python / ROS2)
  │  Modbus RTU over USB Serial
  └─→ MSPM0G3507
        ├─ UART0: RPi通信
        ├─ UART1: モータードライバ通信
        ├─ I2C: IMU (0x23)
        └─ エンコーダ x2
```

RPi から Modbus RTU プロトコルでファームウェアと通信する。
- **Python**: `pymodbus` で直接通信
- **ROS2**: C++ ブリッジノードが Modbus RTU でやり取りし、ROS2 トピックに変換
