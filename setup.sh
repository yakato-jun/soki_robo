#!/usr/bin/env bash
# =============================================================================
# soki_robo セットアップスクリプト
#
# Raspberry Pi (Ubuntu 24.04 Desktop) 用
# リポジトリを clone した直後に実行してください。
#
# 使い方:
#   chmod +x setup.sh
#   ./setup.sh
#
# 実行後、シリアルポートの権限反映のために再ログインが必要です。
# =============================================================================

set -euo pipefail

# --- 色付き出力 ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# --- ルート権限チェック ---
if [[ $EUID -eq 0 ]]; then
    error "root で実行しないでください。sudo は自動で呼ばれます。"
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "============================================"
echo "  soki_robo セットアップ"
echo "  対象: Ubuntu 24.04 (Raspberry Pi)"
echo "============================================"
echo ""

# =============================================================================
# 1. システムパッケージ
# =============================================================================
info "システムパッケージを更新・インストール..."
sudo apt update -qq
sudo apt install -y -qq \
    python3-pip \
    python3-venv \
    git \
    curl \
    software-properties-common \
    > /dev/null

info "システムパッケージ ... OK"

# =============================================================================
# 2. シリアルポート権限 (dialout グループ)
# =============================================================================
if groups "$USER" | grep -q '\bdialout\b'; then
    info "シリアルポート権限 ... OK (dialout グループ所属済み)"
else
    info "dialout グループに追加 (シリアルポートアクセス用)..."
    sudo usermod -aG dialout "$USER"
    warn "権限反映のため、セットアップ完了後に再ログインしてください"
fi

# =============================================================================
# 3. Python 仮想環境 + 依存パッケージ
# =============================================================================
VENV_DIR="${SCRIPT_DIR}/.venv"

if [[ -d "$VENV_DIR" ]]; then
    info "Python 仮想環境 ... 既存 (${VENV_DIR})"
else
    info "Python 仮想環境を作成 (${VENV_DIR})..."
    python3 -m venv "$VENV_DIR"
fi

info "Python パッケージをインストール..."
"$VENV_DIR/bin/pip" install --upgrade pip -q
"$VENV_DIR/bin/pip" install -r "${SCRIPT_DIR}/python/requirements.txt" -q

info "Python 環境 ... OK"
echo "  インストール済みパッケージ:"
"$VENV_DIR/bin/pip" list --format=columns 2>/dev/null | grep -iE "pymodbus|pyserial" | sed 's/^/    /'

# =============================================================================
# 4. ROS2 Jazzy
# =============================================================================
install_ros2() {
    info "ROS2 Jazzy をインストール..."

    # GPG キー
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # リポジトリ追加
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update -qq
    sudo apt install -y -qq \
        ros-jazzy-desktop \
        ros-dev-tools \
        python3-colcon-common-extensions \
        > /dev/null

    info "ROS2 Jazzy ... OK"
}

if command -v ros2 &> /dev/null; then
    ROS2_VERSION=$(ros2 --version 2>/dev/null || echo "unknown")
    info "ROS2 ... 既にインストール済み (${ROS2_VERSION})"
elif dpkg -l ros-jazzy-desktop &> /dev/null 2>&1; then
    info "ROS2 Jazzy ... インストール済み (source /opt/ros/jazzy/setup.bash が必要)"
else
    echo ""
    read -rp "ROS2 Jazzy をインストールしますか？ [y/N]: " ans
    case "$ans" in
        [yY]|[yY][eE][sS])
            install_ros2
            ;;
        *)
            info "ROS2 のインストールをスキップ"
            warn "後で手動でインストールする場合: ./setup.sh を再実行してください"
            ;;
    esac
fi

# =============================================================================
# 5. シリアルポート確認
# =============================================================================
echo ""
info "シリアルポート確認..."
if ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; then
    echo "  上記のポートが見つかりました"
else
    warn "シリアルポートが見つかりません。MCU が接続されているか確認してください。"
fi

# =============================================================================
# 6. 接続テスト
# =============================================================================
echo ""
read -rp "MCU との Modbus 接続テストを実行しますか？ [y/N]: " ans
case "$ans" in
    [yY]|[yY][eE][sS])
        # ポート自動検出
        SERIAL_PORT=""
        for p in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
            if [[ -e "$p" ]]; then
                SERIAL_PORT="$p"
                break
            fi
        done

        if [[ -z "$SERIAL_PORT" ]]; then
            warn "シリアルポートが見つかりません。テストをスキップします。"
        else
            info "接続テスト (${SERIAL_PORT})..."
            "$VENV_DIR/bin/python" -c "
from pymodbus.client import ModbusSerialClient
import struct, sys

client = ModbusSerialClient(port='${SERIAL_PORT}', baudrate=115200, timeout=1)
if not client.connect():
    print('  接続失敗: ポートを開けません')
    sys.exit(1)

# ステータス + ハートビート
result = client.read_holding_registers(0x00, count=2, device_id=1)
if result.isError():
    print('  応答なし: MCU がリセット済みか確認してください')
    client.close()
    sys.exit(1)

status = result.registers[0]
hb = result.registers[1]
imu_ok = bool(status & 0x01)
motor_ok = bool(status & 0x02)
print(f'  STATUS:    IMU_OK={imu_ok}, Motor_OK={motor_ok}')
print(f'  HEARTBEAT: {hb}')

# IMU + Mag
result = client.read_holding_registers(0x10, count=9, device_id=1)
if not result.isError():
    vals = [r if r < 0x8000 else r - 0x10000 for r in result.registers]
    print(f'  ACCEL:     X={vals[0]}, Y={vals[1]}, Z={vals[2]}')
    print(f'  GYRO:      X={vals[3]}, Y={vals[4]}, Z={vals[5]}')
    print(f'  MAG:       X={vals[6]}, Y={vals[7]}, Z={vals[8]}')

# Quaternion
result = client.read_holding_registers(0x20, count=8, device_id=1)
if not result.isError():
    regs = result.registers
    quats = []
    for i in range(4):
        raw = regs[i*2] | (regs[i*2+1] << 16)
        f = struct.unpack('<f', struct.pack('<I', raw))[0]
        quats.append(f)
    print(f'  QUAT:      W={quats[0]:.4f}, X={quats[1]:.4f}, Y={quats[2]:.4f}, Z={quats[3]:.4f}')

# エンコーダ・速度
result = client.read_holding_registers(0x30, count=6, device_id=1)
if not result.isError():
    r = result.registers
    enc_l = (r[0] << 16) | r[1]
    enc_r = (r[2] << 16) | r[3]
    if enc_l >= 0x80000000: enc_l -= 0x100000000
    if enc_r >= 0x80000000: enc_r -= 0x100000000
    spd_l = r[4] if r[4] < 0x8000 else r[4] - 0x10000
    spd_r = r[5] if r[5] < 0x8000 else r[5] - 0x10000
    print(f'  ENCODER:   L={enc_l}, R={enc_r}')
    print(f'  SPEED:     L={spd_l} mm/s, R={spd_r} mm/s')

client.close()
print()
print('  接続テスト OK')
" && info "接続テスト ... OK" || warn "接続テストに失敗しました"
        fi
        ;;
    *)
        info "接続テストをスキップ"
        ;;
esac

# =============================================================================
# 完了
# =============================================================================
echo ""
echo "============================================"
echo "  セットアップ完了"
echo "============================================"
echo ""
echo "使い方:"
echo "  # Python 仮想環境を有効化"
echo "  source .venv/bin/activate"
echo ""
echo "  # ROS2 環境を有効化 (インストール済みの場合)"
echo "  source /opt/ros/jazzy/setup.bash"
echo ""

if ! groups "$USER" | grep -q '\bdialout\b'; then
    echo -e "${YELLOW}注意: シリアルポートの権限反映のため、再ログインしてください${NC}"
    echo ""
fi
