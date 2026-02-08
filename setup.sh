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
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-venv \
    python3-smbus \
    i2c-tools \
    git \
    curl \
    software-properties-common

info "システムパッケージ ... OK"

# =============================================================================
# 2. デバイス権限 (dialout + i2c グループ)
# =============================================================================
for grp in dialout i2c; do
    if groups "$USER" | grep -q "\b${grp}\b"; then
        info "${grp} グループ ... OK (所属済み)"
    else
        info "${grp} グループに追加..."
        sudo usermod -aG "$grp" "$USER"
        warn "権限反映のため、セットアップ完了後に再ログインしてください"
    fi
done

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
"$VENV_DIR/bin/pip" install --upgrade pip
"$VENV_DIR/bin/pip" install -r "${SCRIPT_DIR}/requirements.txt"

info "Python 環境 ... OK"
echo "  インストール済みパッケージ:"
"$VENV_DIR/bin/pip" list --format=columns 2>/dev/null | grep -iE "pymodbus|pyserial|rplidar|numpy|matplotlib|jupyter" | sed 's/^/    /'

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

    sudo apt update
    sudo apt install -y \
        ros-jazzy-desktop \
        ros-dev-tools \
        python3-colcon-common-extensions

    info "ROS2 Jazzy ... OK"
}

install_ros2_packages() {
    info "ROS2 追加パッケージをインストール..."
    sudo apt install -y -qq \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-robot-localization \
        ros-jazzy-rplidar-ros \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-teleop-twist-keyboard \
        libmodbus-dev \
        > /dev/null
    info "ROS2 追加パッケージ ... OK"
}

if command -v ros2 &> /dev/null; then
    ROS2_VERSION=$(ros2 --version 2>/dev/null || echo "unknown")
    info "ROS2 ... 既にインストール済み (${ROS2_VERSION})"
elif dpkg -l ros-jazzy-desktop &> /dev/null 2>&1; then
    info "ROS2 Jazzy ... インストール済み (source /opt/ros/jazzy/setup.bash が必要)"
else
    install_ros2
fi

# ROS2 がインストール済みなら追加パッケージをインストール
if dpkg -l ros-jazzy-desktop &> /dev/null 2>&1; then
    install_ros2_packages
fi

# =============================================================================
# 5. サービス (Jupyter, VNC, noVNC)
# =============================================================================
"${SCRIPT_DIR}/services/install-services.sh"

# =============================================================================
# 6. GPIO UART 有効化 (RPi 5: MCU との Modbus RTU 通信用)
# =============================================================================
info "GPIO UART 設定を確認..."
BOOT_CONFIG="/boot/firmware/config.txt"
if [[ -f "$BOOT_CONFIG" ]]; then
    if grep -q 'dtoverlay=uart0-pi5' "$BOOT_CONFIG"; then
        info "GPIO UART (uart0-pi5) ... 既に有効"
    else
        info "GPIO UART (uart0-pi5) を有効化..."
        sudo sed -i '/^\[all\]$/a dtoverlay=uart0-pi5' "$BOOT_CONFIG"
        warn "GPIO UART を有効化しました。反映には再起動が必要です"
        NEED_REBOOT=1
    fi
else
    warn "$BOOT_CONFIG が見つかりません。手動で dtoverlay=uart0-pi5 を追加してください"
fi

# =============================================================================
# 7. シリアルポート確認
# =============================================================================
echo ""
info "シリアルポート確認..."
if ls /dev/ttyAMA0 2>/dev/null; then
    echo "  /dev/ttyAMA0 (GPIO UART) ... MCU (Modbus RTU)"
fi
if ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; then
    echo "  上記の USB シリアルポートが見つかりました (LiDAR 等)"
fi
if ! ls /dev/ttyAMA0 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null; then
    warn "シリアルポートが見つかりません。再起動が必要か、デバイスが接続されているか確認してください。"
fi

# =============================================================================
# 8. MCU 接続テスト
# =============================================================================
echo ""
read -rp "MCU との Modbus 接続テストを実行しますか？ [y/N]: " ans
case "$ans" in
    [yY]|[yY][eE][sS])
        "$VENV_DIR/bin/python" "${SCRIPT_DIR}/scripts/test_mcu.py" \
            && info "MCU 接続テスト ... OK" \
            || warn "MCU 接続テストに失敗しました"
        ;;
    *)
        info "MCU 接続テストをスキップ"
        ;;
esac

# =============================================================================
# 9. LiDAR 接続テスト
# =============================================================================
echo ""
read -rp "LiDAR (RPLIDAR A1M8) の接続テストを実行しますか？ [y/N]: " ans
case "$ans" in
    [yY]|[yY][eE][sS])
        "$VENV_DIR/bin/python" "${SCRIPT_DIR}/scripts/test_lidar.py" \
            && info "LiDAR 接続テスト ... OK" \
            || warn "LiDAR 接続テストに失敗しました"
        ;;
    *)
        info "LiDAR テストをスキップ"
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
