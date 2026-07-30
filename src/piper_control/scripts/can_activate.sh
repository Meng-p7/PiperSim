#!/usr/bin/env bash

set -Eeuo pipefail

CAN_NAME="${1:-can0}"
BITRATE="${2:-1000000}"
USB_ADDRESS="${3:-}"

if (($# > 3)); then
    printf '[FAIL] 参数过多\n' >&2
    exit 2
fi

if [[ -t 1 && "${TERM:-dumb}" != "dumb" && -z "${NO_COLOR:-}" ]]; then
    green=$'\033[32m'
    yellow=$'\033[33m'
    red=$'\033[31m'
    reset=$'\033[0m'
else
    green=""
    yellow=""
    red=""
    reset=""
fi

info() {
    printf '[INFO] %s\n' "$*"
}

ok() {
    printf '%s[ OK ]%s %s\n' "$green" "$reset" "$*"
}

warn() {
    printf '%s[WARN]%s %s\n' "$yellow" "$reset" "$*" >&2
}

fail() {
    printf '%s[FAIL]%s %s\n' "$red" "$reset" "$*" >&2
}

usage() {
    cat <<'EOF'
用法:
  bash can_activate.sh [接口名] [波特率] [USB bus-info]

示例:
  bash can_activate.sh can0 1000000
  bash can_activate.sh can0 1000000 1-2:1.0
EOF
}

if [[ "$CAN_NAME" == "-h" || "$CAN_NAME" == "--help" ]]; then
    usage
    exit 0
fi
if [[ ! "$CAN_NAME" =~ ^[[:alnum:]_][[:alnum:]_.-]{0,14}$ ]]; then
    fail "非法 CAN 接口名: ${CAN_NAME}"
    exit 2
fi
if [[ ! "$BITRATE" =~ ^[1-9][0-9]*$ ]]; then
    fail "波特率必须是正整数: ${BITRATE}"
    exit 2
fi

elevate=()
configuration_started=false
renamed=false
selected=""
was_up=false
current_bitrate=""

rollback_can() {
    "$configuration_started" || return 0

    warn "配置未完成，正在尝试恢复 ${selected} 的原状态"
    if "$renamed" && ip link show "$CAN_NAME" >/dev/null 2>&1; then
        "${elevate[@]}" ip link set "$CAN_NAME" down >/dev/null 2>&1 || true
        "${elevate[@]}" ip link set "$CAN_NAME" name "$selected" \
            >/dev/null 2>&1 || true
    fi
    if [[ "$current_bitrate" =~ ^[1-9][0-9]*$ ]] &&
       ip link show "$selected" >/dev/null 2>&1; then
        "${elevate[@]}" ip link set "$selected" type can \
            bitrate "$current_bitrate" >/dev/null 2>&1 || true
    fi
    if "$was_up" && ip link show "$selected" >/dev/null 2>&1; then
        "${elevate[@]}" ip link set "$selected" up >/dev/null 2>&1 || true
    fi
}

on_error() {
    local status="$1"
    local line="$2"
    trap - ERR
    fail "CAN 配置在第 ${line} 行失败（退出码: ${status}）"
    rollback_can
    exit "$status"
}

trap 'on_error "$?" "${LINENO}"' ERR

for command_name in ip ethtool; do
    command -v "$command_name" >/dev/null 2>&1 || {
        fail "缺少命令: ${command_name}"
        printf '       修复: sudo apt install iproute2 ethtool can-utils\n' >&2
        exit 1
    }
done

if ((EUID != 0)) && command -v sudo >/dev/null 2>&1; then
    elevate=(sudo)
    "${elevate[@]}" -v
elif ((EUID != 0)); then
    fail "配置 SocketCAN 需要 root 权限，但系统没有 sudo"
    exit 1
fi

mapfile -t interfaces < <(ip -brief link show type can | awk '{print $1}')
if ((${#interfaces[@]} == 0)); then
    fail "未检测到 SocketCAN 接口"
    printf '       检查 USB-CAN 适配器、驱动与 lsmod | grep gs_usb\n' >&2
    exit 1
fi

bus_info() {
    "${elevate[@]}" ethtool -i "$1" 2>/dev/null |
        awk -F': ' '$1 == "bus-info" {print $2; exit}'
}

if [[ -n "$USB_ADDRESS" ]]; then
    for interface in "${interfaces[@]}"; do
        if [[ "$(bus_info "$interface")" == "$USB_ADDRESS" ]]; then
            selected="$interface"
            break
        fi
    done
    if [[ -z "$selected" ]]; then
        fail "找不到 USB bus-info=${USB_ADDRESS} 对应的 CAN 接口"
        exit 1
    fi
elif ((${#interfaces[@]} == 1)); then
    selected="${interfaces[0]}"
else
    fail "检测到多个 CAN 接口，请指定 USB bus-info:"
    for interface in "${interfaces[@]}"; do
        printf '       %-12s %s\n' "$interface" "$(bus_info "$interface")" >&2
    done
    printf '       示例: bash can_activate.sh can0 %s 1-2:1.0\n' "$BITRATE" >&2
    exit 1
fi

info "目标: ${selected} -> ${CAN_NAME}, bitrate=${BITRATE}"

if [[ "$selected" != "$CAN_NAME" ]] && ip link show "$CAN_NAME" >/dev/null 2>&1; then
    fail "目标接口名 ${CAN_NAME} 已存在，拒绝覆盖"
    exit 1
fi

interface_is_up() {
    ip -brief link show "$1" 2>/dev/null |
        awk '$2 == "UP" {found=1} END {exit !found}'
}

if interface_is_up "$selected"; then
    was_up=true
fi
current_bitrate="$(
    ip -details link show "$selected" |
        awk '/ bitrate / {for (i=1; i<=NF; i++) if ($i=="bitrate") {print $(i+1); exit}}'
)"

if "$was_up" && [[ "$current_bitrate" == "$BITRATE" && "$selected" == "$CAN_NAME" ]]; then
    ok "${CAN_NAME} 已处于 UP，bitrate=${BITRATE}"
    exit 0
fi

if "$was_up"; then
    warn "${selected} 当前已 UP（bitrate=${current_bitrate:-unknown}），将重新配置"
fi

configuration_started=true
"${elevate[@]}" ip link set "$selected" down
"${elevate[@]}" ip link set "$selected" type can bitrate "$BITRATE"

if [[ "$selected" != "$CAN_NAME" ]]; then
    "${elevate[@]}" ip link set "$selected" name "$CAN_NAME"
    renamed=true
fi
"${elevate[@]}" ip link set "$CAN_NAME" up

verified_bitrate="$(
    ip -details link show "$CAN_NAME" |
        awk '/ bitrate / {for (i=1; i<=NF; i++) if ($i=="bitrate") {print $(i+1); exit}}'
)"
if ! interface_is_up "$CAN_NAME" ||
   [[ "$verified_bitrate" != "$BITRATE" ]]; then
    fail "配置后验证失败: ${CAN_NAME}, bitrate=${verified_bitrate:-unknown}"
    rollback_can
    exit 1
fi

configuration_started=false
ok "${CAN_NAME} 已激活，bitrate=${verified_bitrate}"
info "状态检查: ip -details link show ${CAN_NAME}"
info "收帧检查: timeout 3 candump ${CAN_NAME}"
