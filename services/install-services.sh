#!/usr/bin/env bash
# =============================================================================
# soki_robo サービス管理スクリプト
#
# 以下のサービスを systemd ユーザーサービスとして管理します:
#   - Jupyter Notebook  (soki-jupyter.service)
#   - TigerVNC Server   (soki-vnc.service)
#   - noVNC Web Client  (soki-novnc.service)
#   - UPS Monitor       (soki-ups.service)
#
# ログアウト後もサービスが維持されるため、RPi の電源を入れるだけで
# ネットワーク経由で各サービスにアクセスできます。
#
# 使い方:
#   ./install-services.sh                # 全サービスインストール
#   ./install-services.sh --uninstall    # 全サービスアンインストール
#   ./install-services.sh --status       # 全サービス状態確認
#   ./install-services.sh --jupyter-only # Jupyter のみインストール
#   ./install-services.sh --vnc-only     # VNC + noVNC のみインストール
#
# ※ root で実行しないでください (systemd ユーザーサービスを使用)
# =============================================================================

set -euo pipefail

# --- 色付き出力 ---
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# --- ルート権限チェック ---
if [[ $EUID -eq 0 ]]; then
    error "root で実行しないでください。systemd ユーザーサービスを使用します。"
fi

# --- パス設定 ---
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

# Jupyter
JUPYTER_SERVICE="soki-jupyter.service"
JUPYTER_TEMPLATE="${SCRIPT_DIR}/soki-jupyter.service.template"

# VNC
VNC_SERVICE="soki-vnc.service"
VNC_TEMPLATE="${SCRIPT_DIR}/soki-vnc.service.template"
NOVNC_SERVICE="soki-novnc.service"
NOVNC_TEMPLATE="${SCRIPT_DIR}/soki-novnc.service.template"
XSTARTUP_SRC="${SCRIPT_DIR}/soki-vnc-xstartup"

# UPS Monitor
UPS_SERVICE="soki-ups.service"
UPS_TEMPLATE="${SCRIPT_DIR}/soki-ups.service.template"

# --- LAN IP 取得 ---
get_lan_ip() {
    local ip
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    if [[ -z "$ip" ]]; then
        ip="<ラズパイのIP>"
    fi
    echo "$ip"
}

# =============================================================================
# Jupyter: ステータス表示
# =============================================================================
status_jupyter() {
    echo ""
    echo -e "  ${CYAN}--- Jupyter Notebook ---${NC}"

    if [[ ! -f "${SYSTEMD_USER_DIR}/${JUPYTER_SERVICE}" ]]; then
        warn "  Jupyter: 未インストール"
        return
    fi

    systemctl --user status "$JUPYTER_SERVICE" --no-pager 2>&1 || true

    if systemctl --user is-active "$JUPYTER_SERVICE" &>/dev/null; then
        local ip
        ip=$(get_lan_ip)
        echo ""
        echo -e "  Jupyter URL: ${CYAN}http://${ip}:8888${NC}"
    fi
}

# =============================================================================
# Jupyter: アンインストール
# =============================================================================
uninstall_jupyter() {
    echo ""
    info "Jupyter サービスをアンインストール..."

    if [[ ! -f "${SYSTEMD_USER_DIR}/${JUPYTER_SERVICE}" ]]; then
        warn "Jupyter: 未インストールです。スキップします。"
        return
    fi

    systemctl --user stop "$JUPYTER_SERVICE" 2>/dev/null || true
    systemctl --user disable "$JUPYTER_SERVICE" 2>/dev/null || true
    rm -f "${SYSTEMD_USER_DIR}/${JUPYTER_SERVICE}"

    info "Jupyter: アンインストール完了"
}

# =============================================================================
# Jupyter: インストール
# =============================================================================
install_jupyter() {
    echo ""
    info "Jupyter サービスをインストール..."

    # --- 前提条件チェック ---
    if [[ ! -f "$JUPYTER_TEMPLATE" ]]; then
        error "テンプレートが見つかりません: ${JUPYTER_TEMPLATE}"
    fi

    if [[ ! -d "${PROJECT_ROOT}/.venv" ]]; then
        error ".venv が見つかりません。先に setup.sh を実行してください。"
    fi

    if [[ ! -x "${PROJECT_ROOT}/.venv/bin/jupyter-notebook" ]]; then
        error "jupyter-notebook が見つかりません。.venv に jupyter がインストールされているか確認してください。"
    fi

    info "Jupyter: 前提条件 OK"

    # --- systemd ユーザーディレクトリ作成 ---
    mkdir -p "$SYSTEMD_USER_DIR"

    # --- テンプレートからサービスファイル生成 ---
    sed "s|@@PROJECT_ROOT@@|${PROJECT_ROOT}|g" "$JUPYTER_TEMPLATE" \
        > "${SYSTEMD_USER_DIR}/${JUPYTER_SERVICE}"
    info "  → ${SYSTEMD_USER_DIR}/${JUPYTER_SERVICE}"

    # --- 有効化 + 起動 ---
    systemctl --user enable --now "$JUPYTER_SERVICE"
    info "Jupyter: インストール完了"
}

# =============================================================================
# VNC: パッケージチェック + インストール
# =============================================================================
check_vnc_deps() {
    local missing=()

    for pkg in xfce4 tigervnc-standalone-server novnc python3-websockify; do
        if ! dpkg -l "$pkg" 2>/dev/null | grep -q '^ii'; then
            missing+=("$pkg")
        fi
    done

    if [[ ${#missing[@]} -gt 0 ]]; then
        info "VNC に必要なパッケージをインストールします: ${missing[*]}"
        sudo apt-get update -qq
        sudo apt-get install -y "${missing[@]}"
    else
        info "VNC: 必須パッケージ OK"
    fi
}

# =============================================================================
# VNC: ステータス表示
# =============================================================================
status_vnc() {
    echo ""
    echo -e "  ${CYAN}--- TigerVNC Server ---${NC}"

    if [[ ! -f "${SYSTEMD_USER_DIR}/${VNC_SERVICE}" ]]; then
        warn "  VNC: 未インストール"
    else
        systemctl --user status "$VNC_SERVICE" --no-pager 2>&1 || true
    fi

    echo ""
    echo -e "  ${CYAN}--- noVNC Web Client ---${NC}"

    if [[ ! -f "${SYSTEMD_USER_DIR}/${NOVNC_SERVICE}" ]]; then
        warn "  noVNC: 未インストール"
    else
        systemctl --user status "$NOVNC_SERVICE" --no-pager 2>&1 || true
    fi

    if systemctl --user is-active "$NOVNC_SERVICE" &>/dev/null; then
        local ip
        ip=$(get_lan_ip)
        echo ""
        echo -e "  noVNC URL: ${CYAN}http://${ip}:6080/vnc.html${NC}"
    fi
}

# =============================================================================
# VNC: アンインストール
# =============================================================================
uninstall_vnc() {
    echo ""
    info "VNC + noVNC サービスをアンインストール..."

    # noVNC を先に停止 (VNC に依存しているため)
    if [[ -f "${SYSTEMD_USER_DIR}/${NOVNC_SERVICE}" ]]; then
        systemctl --user stop "$NOVNC_SERVICE" 2>/dev/null || true
        systemctl --user disable "$NOVNC_SERVICE" 2>/dev/null || true
        rm -f "${SYSTEMD_USER_DIR}/${NOVNC_SERVICE}"
        info "noVNC: アンインストール完了"
    else
        warn "noVNC: 未インストールです。スキップします。"
    fi

    if [[ -f "${SYSTEMD_USER_DIR}/${VNC_SERVICE}" ]]; then
        systemctl --user stop "$VNC_SERVICE" 2>/dev/null || true
        systemctl --user disable "$VNC_SERVICE" 2>/dev/null || true
        rm -f "${SYSTEMD_USER_DIR}/${VNC_SERVICE}"
        info "VNC: アンインストール完了"
    else
        warn "VNC: 未インストールです。スキップします。"
    fi
}

# =============================================================================
# VNC: インストール
# =============================================================================
install_vnc() {
    echo ""
    info "VNC + noVNC サービスをインストール..."

    # --- テンプレート存在チェック ---
    if [[ ! -f "$VNC_TEMPLATE" ]]; then
        error "テンプレートが見つかりません: ${VNC_TEMPLATE}"
    fi
    if [[ ! -f "$NOVNC_TEMPLATE" ]]; then
        error "テンプレートが見つかりません: ${NOVNC_TEMPLATE}"
    fi
    if [[ ! -f "$XSTARTUP_SRC" ]]; then
        error "xstartup が見つかりません: ${XSTARTUP_SRC}"
    fi

    # --- パッケージチェック ---
    check_vnc_deps

    # --- xstartup を ~/.vnc/ に配置 ---
    mkdir -p "${HOME}/.vnc"
    cp "$XSTARTUP_SRC" "${HOME}/.vnc/xstartup"
    chmod +x "${HOME}/.vnc/xstartup"
    info "xstartup を配置: ${HOME}/.vnc/xstartup"

    # --- systemd ユーザーディレクトリ作成 ---
    mkdir -p "$SYSTEMD_USER_DIR"

    # --- テンプレートからサービスファイル生成 ---
    sed "s|@@PROJECT_ROOT@@|${PROJECT_ROOT}|g" "$VNC_TEMPLATE" \
        > "${SYSTEMD_USER_DIR}/${VNC_SERVICE}"
    info "  → ${SYSTEMD_USER_DIR}/${VNC_SERVICE}"

    # noVNC テンプレートにはプレースホルダがないが、将来の拡張性のため sed を通す
    sed "s|@@PROJECT_ROOT@@|${PROJECT_ROOT}|g" "$NOVNC_TEMPLATE" \
        > "${SYSTEMD_USER_DIR}/${NOVNC_SERVICE}"
    info "  → ${SYSTEMD_USER_DIR}/${NOVNC_SERVICE}"

    # --- 有効化 + 起動 ---
    systemctl --user enable --now "$VNC_SERVICE"
    systemctl --user enable --now "$NOVNC_SERVICE"
    info "VNC + noVNC: インストール完了"
}

# =============================================================================
# UPS Monitor: ステータス表示
# =============================================================================
status_ups() {
    echo ""
    echo -e "  ${CYAN}--- UPS Battery Monitor ---${NC}"

    if [[ ! -f "${SYSTEMD_USER_DIR}/${UPS_SERVICE}" ]]; then
        warn "  UPS Monitor: 未インストール"
        return
    fi

    systemctl --user status "$UPS_SERVICE" --no-pager 2>&1 || true
}

# =============================================================================
# UPS Monitor: アンインストール
# =============================================================================
uninstall_ups() {
    echo ""
    info "UPS Monitor サービスをアンインストール..."

    if [[ ! -f "${SYSTEMD_USER_DIR}/${UPS_SERVICE}" ]]; then
        warn "UPS Monitor: 未インストールです。スキップします。"
        return
    fi

    systemctl --user stop "$UPS_SERVICE" 2>/dev/null || true
    systemctl --user disable "$UPS_SERVICE" 2>/dev/null || true
    rm -f "${SYSTEMD_USER_DIR}/${UPS_SERVICE}"

    info "UPS Monitor: アンインストール完了"
}

# =============================================================================
# UPS Monitor: インストール
# =============================================================================
install_ups() {
    echo ""
    info "UPS Monitor サービスをインストール..."

    if [[ ! -f "$UPS_TEMPLATE" ]]; then
        error "テンプレートが見つかりません: ${UPS_TEMPLATE}"
    fi

    if [[ ! -d "${PROJECT_ROOT}/.venv" ]]; then
        error ".venv が見つかりません。先に setup.sh を実行してください。"
    fi

    # I2C デバイス確認
    if [[ ! -e /dev/i2c-1 ]]; then
        warn "I2C デバイス (/dev/i2c-1) が見つかりません。UPS Monitor をスキップします。"
        return
    fi

    info "UPS Monitor: 前提条件 OK"

    mkdir -p "$SYSTEMD_USER_DIR"

    sed "s|@@PROJECT_ROOT@@|${PROJECT_ROOT}|g" "$UPS_TEMPLATE" \
        > "${SYSTEMD_USER_DIR}/${UPS_SERVICE}"
    info "  → ${SYSTEMD_USER_DIR}/${UPS_SERVICE}"

    systemctl --user enable --now "$UPS_SERVICE"
    info "UPS Monitor: インストール完了"
}

# =============================================================================
# 統合: ステータス表示
# =============================================================================
show_status() {
    echo ""
    echo "============================================"
    echo "  soki_robo サービス状態"
    echo "============================================"

    status_jupyter
    status_vnc
    status_ups

    echo ""
}

# =============================================================================
# 統合: アンインストール
# =============================================================================
do_uninstall() {
    echo ""
    echo "============================================"
    echo "  soki_robo サービス アンインストール"
    echo "============================================"

    uninstall_ups
    uninstall_vnc
    uninstall_jupyter

    info "systemd を再読み込み..."
    systemctl --user daemon-reload

    # NOTE: loginctl disable-linger は行わない (他サービスが使う可能性があるため)

    echo ""
    info "全サービスのアンインストール完了"
    echo ""
}

# =============================================================================
# 統合: インストール
# =============================================================================
do_install() {
    echo ""
    echo "============================================"
    echo "  soki_robo サービス インストール"
    echo "============================================"
    echo ""

    info "前提条件を確認..."

    install_jupyter
    install_vnc
    install_ups

    # --- systemd 再読み込み ---
    info "systemd を再読み込み..."
    systemctl --user daemon-reload

    # --- loginctl enable-linger ---
    info "linger を有効化 (ログアウト後もサービスを維持)..."
    loginctl enable-linger "$USER" 2>/dev/null || warn "loginctl enable-linger に失敗しました (管理者に確認してください)"

    # --- 完了メッセージ ---
    local ip
    ip=$(get_lan_ip)

    echo ""
    echo "============================================"
    echo "  インストール完了"
    echo "============================================"
    echo ""
    echo "  RPi の電源を入れるだけで各サービスにアクセスできます。"
    echo ""
    echo -e "  Jupyter URL: ${CYAN}http://${ip}:8888${NC}"
    echo -e "  noVNC URL:   ${CYAN}http://${ip}:6080/vnc.html${NC}"
    echo ""
    echo "  便利なコマンド:"
    echo "    状態確認:  $0 --status"
    echo "    ログ確認:  journalctl --user -u ${JUPYTER_SERVICE} -f"
    echo "              journalctl --user -u ${VNC_SERVICE} -f"
    echo "              journalctl --user -u ${NOVNC_SERVICE} -f"
    echo "              journalctl --user -u ${UPS_SERVICE} -f"
    echo "    再起動:    systemctl --user restart ${JUPYTER_SERVICE}"
    echo "              systemctl --user restart ${VNC_SERVICE} ${NOVNC_SERVICE}"
    echo "              systemctl --user restart ${UPS_SERVICE}"
    echo ""
}

# =============================================================================
# 単体: Jupyter のみ
# =============================================================================
do_install_jupyter_only() {
    echo ""
    echo "============================================"
    echo "  Jupyter サービス インストール"
    echo "============================================"
    echo ""

    install_jupyter

    systemctl --user daemon-reload
    loginctl enable-linger "$USER" 2>/dev/null || warn "loginctl enable-linger に失敗しました"

    local ip
    ip=$(get_lan_ip)

    echo ""
    info "インストール完了"
    echo -e "  Jupyter URL: ${CYAN}http://${ip}:8888${NC}"
    echo ""
}

# =============================================================================
# 単体: VNC のみ
# =============================================================================
do_install_vnc_only() {
    echo ""
    echo "============================================"
    echo "  VNC + noVNC サービス インストール"
    echo "============================================"
    echo ""

    install_vnc

    systemctl --user daemon-reload
    loginctl enable-linger "$USER" 2>/dev/null || warn "loginctl enable-linger に失敗しました"

    local ip
    ip=$(get_lan_ip)

    echo ""
    info "インストール完了"
    echo -e "  noVNC URL: ${CYAN}http://${ip}:6080/vnc.html${NC}"
    echo ""
}

# =============================================================================
# 単体: Jupyter アンインストール
# =============================================================================
do_uninstall_jupyter_only() {
    echo ""
    echo "============================================"
    echo "  Jupyter サービス アンインストール"
    echo "============================================"

    uninstall_jupyter
    systemctl --user daemon-reload

    echo ""
    info "Jupyter アンインストール完了"
    echo ""
}

# =============================================================================
# 単体: VNC アンインストール
# =============================================================================
do_uninstall_vnc_only() {
    echo ""
    echo "============================================"
    echo "  VNC + noVNC サービス アンインストール"
    echo "============================================"

    uninstall_vnc
    systemctl --user daemon-reload

    echo ""
    info "VNC + noVNC アンインストール完了"
    echo ""
}

# =============================================================================
# メイン
# =============================================================================
case "${1:-}" in
    --status|-s)
        show_status
        ;;
    --uninstall|--remove|-u)
        case "${2:-}" in
            --jupyter-only) do_uninstall_jupyter_only ;;
            --vnc-only)     do_uninstall_vnc_only ;;
            "")             do_uninstall ;;
            *)              error "不明なオプション: $2" ;;
        esac
        ;;
    --jupyter-only)
        do_install_jupyter_only
        ;;
    --vnc-only)
        do_install_vnc_only
        ;;
    --help|-h)
        echo "使い方: $0 [オプション]"
        echo ""
        echo "オプション:"
        echo "  (なし)              全サービスインストール (Jupyter + VNC)"
        echo "  --jupyter-only      Jupyter のみインストール"
        echo "  --vnc-only          VNC + noVNC のみインストール"
        echo "  --status, -s        全サービス状態確認"
        echo "  --uninstall         全サービスアンインストール"
        echo "  --uninstall --jupyter-only  Jupyter のみアンインストール"
        echo "  --uninstall --vnc-only      VNC + noVNC のみアンインストール"
        echo "  --help, -h          このヘルプを表示"
        ;;
    "")
        do_install
        ;;
    *)
        error "不明なオプション: $1 (--help でヘルプを表示)"
        ;;
esac
