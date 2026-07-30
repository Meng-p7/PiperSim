#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UPSTREAM_SCRIPT="${PROJECT_ROOT}/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

piper_header "安装 Orbbec 宿主机 udev 规则"

if [[ -f /.dockerenv ]]; then
    piper_error "udev 规则必须安装在 Ubuntu 宿主机，不能在容器内安装"
    exit 1
fi

if [[ ! -f "$UPSTREAM_SCRIPT" ]]; then
    piper_error "Orbbec 子模块未初始化"
    piper_fix "git submodule update --init --recursive"
    exit 1
fi

command -v udevadm >/dev/null 2>&1 || {
    piper_error "宿主机缺少 udevadm"
    exit 1
}

piper_info "将调用 Orbbec 官方规则安装脚本，需要 sudo 权限"
sudo bash "$UPSTREAM_SCRIPT"
piper_ok "udev 规则已加载；请重新插拔相机"
