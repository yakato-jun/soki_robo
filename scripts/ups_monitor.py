#!/usr/bin/env python3
"""UPS Plus (EP-0136) バッテリー監視デーモン

バッテリー電圧を定期的に監視し、低下時に安全シャットダウンを実行します。

I2C デバイス:
  0x17: UPS MCU (バッテリー電圧, 容量)
  0x40: INA219 供給電力モニター
  0x45: INA219 バッテリーモニター

UPS MCU レジスタマップ (バイトアドレス):
  0x01-0x02: MCU 内部電圧 (2400-3600 mV)
  0x03-0x04: Pogopin 電圧 (0-5500 mV)
  0x05-0x06: バッテリー端子電圧 (0-4500 mV)
  0x07-0x08: USB-C 充電ポート電圧 (0-13500 mV)
  0x09-0x0A: MicroUSB 充電ポート電圧 (0-13500 mV)
  0x0B-0x0C: バッテリー温度 (-20〜65 ℃)
  0x13-0x14: バッテリー残量 (0-100 %)
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
PROTECT_VOLTAGE_MV = 3500   # この電圧以下でシャットダウン (放電時のみ)
WARN_VOLTAGE_MV = 3700      # この電圧以下で警告開始
CHECK_INTERVAL_S = 30       # 監視間隔 (秒)
LOW_COUNT_THRESHOLD = 3     # 連続N回低電圧でシャットダウン

MCU_ADDR = 0x17
I2C_BUS = 1

# UPS MCU レジスタアドレス (バイトアドレス)
REG_BATT_VOLTAGE = 0x05     # バッテリー端子電圧 (mV)
REG_USB_C_VOLTAGE = 0x07    # USB-C 充電ポート電圧 (mV)
REG_MICRO_USB_VOLTAGE = 0x09  # MicroUSB 充電ポート電圧 (mV)
REG_BATT_CAPACITY = 0x13    # バッテリー残量 (%)


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
            batt_v_mv = bus.read_word_data(MCU_ADDR, REG_BATT_VOLTAGE)
            batt_cap = bus.read_word_data(MCU_ADDR, REG_BATT_CAPACITY)
            usb_c_mv = bus.read_word_data(MCU_ADDR, REG_USB_C_VOLTAGE)
            micro_usb_mv = bus.read_word_data(MCU_ADDR, REG_MICRO_USB_VOLTAGE)
            supply_v = ina_supply.voltage()
            batt_i = ina_batt.current()

            charging = "充電中" if batt_i < 0 else "放電中"
            log.info(
                "Battery=%dmV Cap=%d%% Supply=%.2fV I=%.0fmA (%s) USB-C=%dmV MicroUSB=%dmV",
                batt_v_mv, batt_cap, supply_v, batt_i, charging,
                usb_c_mv, micro_usb_mv,
            )

            # 外部電源が接続されている場合はシャットダウンしない
            has_external_power = usb_c_mv > 1000 or micro_usb_mv > 1000

            if batt_v_mv <= PROTECT_VOLTAGE_MV and not has_external_power:
                low_count += 1
                log.warning(
                    "低電圧検出: %d mV (連続 %d/%d)",
                    batt_v_mv, low_count, LOW_COUNT_THRESHOLD,
                )
                if low_count >= LOW_COUNT_THRESHOLD:
                    shutdown()
            elif batt_v_mv <= WARN_VOLTAGE_MV and not has_external_power:
                low_count = 0
                log.warning("バッテリー残量注意: %d mV", batt_v_mv)
            else:
                low_count = 0

        except Exception as e:
            log.error("読み取りエラー: %s", e)

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
