# soki_robo

自律走行ロボットプロジェクト。

## ディレクトリ構成

```
soki_robo/
├── requirements.txt  # Python 依存パッケージ (.venv 用)
├── setup.sh          # RPi セットアップスクリプト
├── lessons/          # レッスン（段階的に学習）
├── scripts/          # Python スクリプト (テスト・デーモン)
│   ├── test_mcu.py       # MCU 接続テスト
│   ├── test_lidar.py     # LiDAR 接続テスト
│   └── ups_monitor.py    # UPS バッテリー監視デーモン
├── examples/         # Jupyter ノートブック
│   ├── motor_test.ipynb   # モーター動作テスト
│   └── ups_monitor.ipynb  # UPS バッテリーモニター
├── services/         # systemd サービス管理
│   ├── install-services.sh
│   └── *.service.template
├── ros2_ws/          # ROS2 ワークスペース
│   └── src/
│       ├── soki_bringup/      # 起動・設定
│       ├── soki_description/  # URDF・モデル
│       ├── soki_hardware/     # ハードウェアインターフェース
│       └── soki_navigation/   # ナビゲーション
└── firmware/         # MSPM0 ファームウェア
    ├── soki_main/   # メインファームウェア（Modbus RTU スレーブ）
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
  │  Modbus RTU over GPIO UART (PA25/PA26 ↔ GPIO14/GPIO15)
  └─→ MSPM0G3507
        ├─ UART3 (PA26 TX / PA25 RX): RPi通信
        ├─ UART1 (PB6 TX / PB7 RX): モータードライバ通信
        ├─ I2C: IMU (0x23)
        └─ エンコーダ x2
```

RPi から Modbus RTU プロトコルでファームウェアと通信する。
- **Python**: `pymodbus` で直接通信
- **ROS2**: C++ ブリッジノードが Modbus RTU でやり取りし、ROS2 トピックに変換
