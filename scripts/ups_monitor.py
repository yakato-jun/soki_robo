#!/usr/bin/env python3
"""UPS Plus (EP-0136) バッテリー監視デーモン

バッテリー電圧を定期的に監視し、低下時に安全シャットダウンを実行します。

公式リファレンス: https://github.com/geeekpi/upsplus

I2C デバイス:
  0x17: UPS MCU (バッテリー電圧, 容量)
  0x40: INA219 供給電力モニター
  0x45: INA219 バッテリーモニター

UPS MCU レジスタマップ (バイトアドレス, read_byte_data で1バイトずつ読み取り):
  1-2:   MCU 内部電圧 (mV)
  3-4:   RPi 出力電圧 (mV)
  5-6:   バッテリー端子電圧 (mV) ※充電中は不正確
  7-8:   USB-C 充電ポート電圧 (mV)
  9-10:  MicroUSB 充電ポート電圧 (mV)
  11-12: バッテリー温度 (℃)
  17-18: 保護電圧 (mV, RW)
  19-20: バッテリー残量 (%)
  23:    電源状態 (1=正常)
  24:    シャットダウンカウントダウン (秒, RW)
  25:    外部電源復帰時自動起動 (0/1, RW)
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
PROTECT_VOLTAGE_MV = 3500   # この電圧以下でシャットダウン (非充電時のみ)
WARN_VOLTAGE_MV = 3700      # この電圧以下で警告開始
CHECK_INTERVAL_S = 30       # 監視間隔 (秒)
LOW_COUNT_THRESHOLD = 3     # 連続N回低電圧でシャットダウン

MCU_ADDR = 0x17
I2C_BUS = 1


def read_mcu_16(bus, reg_lo):
    """UPS MCU から 16bit LE 値を読み出す (公式と同じ read_byte_data 方式)"""
    lo = bus.read_byte_data(MCU_ADDR, reg_lo)
    hi = bus.read_byte_data(MCU_ADDR, reg_lo + 1)
    return (hi << 8) | lo


def shutdown(bus):
    """安全シャットダウン (公式と同じくレジスタ24にカウントダウンを書き込む)"""
    log.warning("バッテリー低下 — シャットダウンを実行します")
    bus.write_byte_data(MCU_ADDR, 24, 240)
    subprocess.run(["sudo", "sync"], check=False)
    subprocess.run(["sudo", "halt"], check=False)
    while True:
        time.sleep(10)


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
            # MCU レジスタ読み取り (公式と同じ read_byte_data 方式)
            batt_v_mv = read_mcu_16(bus, 5)      # bytes 5-6: バッテリー端子電圧
            batt_cap = read_mcu_16(bus, 19)       # bytes 19-20: 残量 (%)
            usb_c_mv = read_mcu_16(bus, 7)        # bytes 7-8: USB-C 電圧
            micro_usb_mv = read_mcu_16(bus, 9)    # bytes 9-10: MicroUSB 電圧

            # INA219 読み取り
            supply_v = ina_supply.voltage()
            batt_voltage = ina_batt.voltage()
            batt_i = ina_batt.current()

            # 充電判定 (公式準拠: USB-C or MicroUSB > 4000mV で充電中)
            if usb_c_mv > 4000:
                charge_src = "充電中(USB-C)"
            elif micro_usb_mv > 4000:
                charge_src = "充電中(MicroUSB)"
            else:
                charge_src = "非充電"

            log.info(
                "Battery=%dmV(INA:%.2fV) Cap=%d%% Supply=%.2fV I=%.0fmA %s",
                batt_v_mv, batt_voltage, batt_cap, supply_v, batt_i, charge_src,
            )

            # シャットダウン判定 (公式準拠: 非充電時のみ INA219 電圧で判定)
            is_charging = usb_c_mv > 4000 or micro_usb_mv > 4000

            if not is_charging and str(batt_voltage) != "0.0":
                batt_mv_ina = batt_voltage * 1000
                if batt_mv_ina < PROTECT_VOLTAGE_MV + 200:
                    low_count += 1
                    log.warning(
                        "低電圧検出: %.0f mV (連続 %d/%d)",
                        batt_mv_ina, low_count, LOW_COUNT_THRESHOLD,
                    )
                    if low_count >= LOW_COUNT_THRESHOLD:
                        shutdown(bus)
                elif batt_mv_ina < WARN_VOLTAGE_MV:
                    low_count = 0
                    log.warning("バッテリー残量注意: %.0f mV", batt_mv_ina)
                else:
                    low_count = 0
            else:
                low_count = 0

        except Exception as e:
            log.error("読み取りエラー: %s", e)

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
