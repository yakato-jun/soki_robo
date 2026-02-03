#!/usr/bin/env python3
"""LiDAR (RPLIDAR A1M8) 接続テスト

使い方:
  python3 utils/test_lidar.py [ポート]

ポート省略時は /dev/ttyUSB1, /dev/ttyUSB0, /dev/ttyUSB2 を順に探します。
"""

import sys
import os


def find_serial_port():
    for p in ["/dev/ttyUSB1", "/dev/ttyUSB0", "/dev/ttyUSB2"]:
        if os.path.exists(p):
            return p
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_serial_port()
    if port is None:
        print("  シリアルポートが見つかりません")
        sys.exit(1)

    from rplidar import RPLidar

    print(f"  ポート: {port}")
    lidar = RPLidar(port=port, baudrate=115200, timeout=1)
    try:
        info = lidar.get_info()
        print(f'  Model:     {info["model"]}')
        print(f'  Firmware:  {info["firmware"]}')
        print(f'  Hardware:  {info["hardware"]}')

        health = lidar.get_health()
        print(f"  Health:    {health[0]} (code: {health[1]})")
        if health[0] == "Error":
            print("  センサがエラー状態です。リセットが必要です。")
            sys.exit(1)

        # 1スキャンだけ取得して確認
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
            valid = [(q, a, d) for q, a, d in scan if d > 0]
            print(f"  Scan:      {len(valid)} valid points")
            if valid:
                min_d = min(d for _, _, d in valid)
                max_d = max(d for _, _, d in valid)
                print(f"  Range:     {min_d:.0f} - {max_d:.0f} mm")
            break

        print()
        print("  LiDAR 接続テスト OK")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


if __name__ == "__main__":
    main()
