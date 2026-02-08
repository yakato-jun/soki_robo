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


def do_shutdown():
    """安全シャットダウン (公式と同じくレジスタ24にカウントダウンを書き込む)"""
    log.warning("バッテリー低下 — シャットダウンを実行します")
    import smbus2
    bus = smbus2.SMBus(I2C_BUS)
    try:
        bus.write_byte_data(MCU_ADDR, 24, 240)
    finally:
        bus.close()
    subprocess.run(["sudo", "sync"], check=False)
    subprocess.run(["sudo", "halt"], check=False)
    while True:
        time.sleep(10)


def read_status():
    """I2C バスを開いて全センサーを読み取り、閉じて返す"""
    import smbus2
    from ina219 import INA219

    bus = smbus2.SMBus(I2C_BUS)
    try:
        batt_v_mv = read_mcu_16(bus, 5)      # bytes 5-6: バッテリー端子電圧
        batt_cap = read_mcu_16(bus, 19)       # bytes 19-20: 残量 (%)
        usb_c_mv = read_mcu_16(bus, 7)        # bytes 7-8: USB-C 電圧
        micro_usb_mv = read_mcu_16(bus, 9)    # bytes 9-10: MicroUSB 電圧
    finally:
        bus.close()

    ina_supply = INA219(0.00725, busnum=I2C_BUS, address=0x40)
    ina_supply.configure()
    supply_v = ina_supply.voltage()
    ina_supply._i2c.close()

    ina_batt = INA219(0.005, busnum=I2C_BUS, address=0x45)
    ina_batt.configure()
    batt_voltage = ina_batt.voltage()
    batt_i = ina_batt.current()
    ina_batt._i2c.close()

    return {
        "batt_v_mv": batt_v_mv,
        "batt_cap": batt_cap,
        "usb_c_mv": usb_c_mv,
        "micro_usb_mv": micro_usb_mv,
        "supply_v": supply_v,
        "batt_voltage": batt_voltage,
        "batt_i": batt_i,
    }


def main():
    log.info("UPS 監視開始 (保護電圧: %d mV, 間隔: %d 秒)", PROTECT_VOLTAGE_MV, CHECK_INTERVAL_S)

    low_count = 0

    while True:
        try:
            s = read_status()

            # 充電判定 (公式準拠: USB-C or MicroUSB > 4000mV で充電中)
            if s["usb_c_mv"] > 4000:
                charge_src = "充電中(USB-C)"
            elif s["micro_usb_mv"] > 4000:
                charge_src = "充電中(MicroUSB)"
            else:
                charge_src = "非充電"

            log.info(
                "Battery=%dmV(INA:%.2fV) Cap=%d%% Supply=%.2fV I=%.0fmA %s",
                s["batt_v_mv"], s["batt_voltage"], s["batt_cap"],
                s["supply_v"], s["batt_i"], charge_src,
            )

            # シャットダウン判定 (公式準拠: 非充電時のみ INA219 電圧で判定)
            is_charging = s["usb_c_mv"] > 4000 or s["micro_usb_mv"] > 4000

            if not is_charging and str(s["batt_voltage"]) != "0.0":
                batt_mv_ina = s["batt_voltage"] * 1000
                if batt_mv_ina < PROTECT_VOLTAGE_MV + 200:
                    low_count += 1
                    log.warning(
                        "低電圧検出: %.0f mV (連続 %d/%d)",
                        batt_mv_ina, low_count, LOW_COUNT_THRESHOLD,
                    )
                    if low_count >= LOW_COUNT_THRESHOLD:
                        do_shutdown()
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
