#!/usr/bin/env python3
"""UPS Plus (EP-0136) バッテリー監視デーモン

バッテリー電圧を定期的に監視し、低下時に安全シャットダウンを実行します。

I2C デバイス:
  0x17: UPS MCU (バッテリー電圧, 容量)
  0x40: INA219 供給電力モニター
  0x45: INA219 バッテリーモニター
"""

import time
import subprocess
import sys
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [UPS] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger(__name__)

# --- 設定 ---
PROTECT_VOLTAGE_MV = 3500   # この電圧以下で即シャットダウン
WARN_VOLTAGE_MV = 3700      # この電圧以下で警告開始
CHECK_INTERVAL_S = 30       # 監視間隔 (秒)
LOW_COUNT_THRESHOLD = 3     # 連続N回低電圧でシャットダウン

MCU_ADDR = 0x17
I2C_BUS = 1


def read_mcu_16(bus, reg):
    """UPS MCU から 16bit リトルエンディアン値を読み出す"""
    lo = bus.read_byte_data(MCU_ADDR, reg)
    hi = bus.read_byte_data(MCU_ADDR, reg + 1)
    return (hi << 8) | lo


def shutdown():
    """安全シャットダウン"""
    log.warning("バッテリー低下 — シャットダウンを実行します")
    subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)
    sys.exit(0)


def main():
    import smbus2
    from ina219 import INA219

    bus = smbus2.SMBus(I2C_BUS)

    ina_supply = INA219(0.00725, busnum=I2C_BUS, address=0x40)
    ina_supply.configure()

    ina_batt = INA219(0.005, busnum=I2C_BUS, address=0x45)
    ina_batt.configure()

    log.info("UPS 監視開始 (保護電圧: %d mV, 間隔: %d 秒)", PROTECT_VOLTAGE_MV, CHECK_INTERVAL_S)

    low_count = 0

    while True:
        try:
            batt_v_mv = read_mcu_16(bus, 1)
            batt_cap = read_mcu_16(bus, 19)
            supply_v = ina_supply.voltage()
            batt_i = ina_batt.current()

            charging = "充電中" if batt_i < 0 else "放電中"
            log.info(
                "Battery=%dmV Cap=%d%% Supply=%.2fV I=%.0fmA (%s)",
                batt_v_mv, batt_cap, supply_v, batt_i, charging,
            )

            if batt_v_mv <= PROTECT_VOLTAGE_MV:
                low_count += 1
                log.warning(
                    "低電圧検出: %d mV (連続 %d/%d)",
                    batt_v_mv, low_count, LOW_COUNT_THRESHOLD,
                )
                if low_count >= LOW_COUNT_THRESHOLD:
                    shutdown()
            elif batt_v_mv <= WARN_VOLTAGE_MV:
                low_count = 0
                log.warning("バッテリー残量注意: %d mV", batt_v_mv)
            else:
                low_count = 0

        except Exception as e:
            log.error("読み取りエラー: %s", e)

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
