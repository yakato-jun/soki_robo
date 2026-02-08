#!/usr/bin/env python3
"""MCU (MSPM0G3507) Modbus RTU 接続テスト

使い方:
  python3 utils/test_mcu.py [ポート]

ポート省略時は /dev/ttyAMA0 (GPIO UART) を優先し、なければ USB シリアルを探します。
"""

import struct
import sys
import os


def find_serial_port():
    for p in ["/dev/ttyAMA0", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]:
        if os.path.exists(p):
            return p
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_serial_port()
    if port is None:
        print("  シリアルポートが見つかりません")
        sys.exit(1)

    from pymodbus.client import ModbusSerialClient

    print(f"  ポート: {port}")
    client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
    if not client.connect():
        print("  接続失敗: ポートを開けません")
        sys.exit(1)

    # ステータス + ハートビート
    result = client.read_holding_registers(0x00, count=2, device_id=1)
    if result.isError():
        print("  応答なし: MCU がリセット済みか確認してください")
        client.close()
        sys.exit(1)

    status = result.registers[0]
    hb = result.registers[1]
    imu_ok = bool(status & 0x01)
    motor_ok = bool(status & 0x02)
    print(f"  STATUS:    IMU_OK={imu_ok}, Motor_OK={motor_ok}")
    print(f"  HEARTBEAT: {hb}")

    # IMU + Mag
    result = client.read_holding_registers(0x10, count=9, device_id=1)
    if not result.isError():
        vals = [r if r < 0x8000 else r - 0x10000 for r in result.registers]
        print(f"  ACCEL:     X={vals[0]}, Y={vals[1]}, Z={vals[2]}")
        print(f"  GYRO:      X={vals[3]}, Y={vals[4]}, Z={vals[5]}")
        print(f"  MAG:       X={vals[6]}, Y={vals[7]}, Z={vals[8]}")

    # Quaternion
    result = client.read_holding_registers(0x20, count=8, device_id=1)
    if not result.isError():
        regs = result.registers
        quats = []
        for i in range(4):
            raw = regs[i * 2] | (regs[i * 2 + 1] << 16)
            f = struct.unpack("<f", struct.pack("<I", raw))[0]
            quats.append(f)
        print(
            f"  QUAT:      W={quats[0]:.4f}, X={quats[1]:.4f}, Y={quats[2]:.4f}, Z={quats[3]:.4f}"
        )

    # エンコーダ・速度
    result = client.read_holding_registers(0x30, count=6, device_id=1)
    if not result.isError():
        r = result.registers
        enc_l = (r[0] << 16) | r[1]
        enc_r = (r[2] << 16) | r[3]
        if enc_l >= 0x80000000:
            enc_l -= 0x100000000
        if enc_r >= 0x80000000:
            enc_r -= 0x100000000
        spd_l = r[4] if r[4] < 0x8000 else r[4] - 0x10000
        spd_r = r[5] if r[5] < 0x8000 else r[5] - 0x10000
        print(f"  ENCODER:   L={enc_l}, R={enc_r}")
        print(f"  SPEED:     L={spd_l} mm/s, R={spd_r} mm/s")

    client.close()
    print()
    print("  接続テスト OK")


if __name__ == "__main__":
    main()
