#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/scripts/lib/common.sh"

trap 'piper_error "相机启动失败（第 ${LINENO} 行，退出码: $?）"' ERR

piper_header "Orbbec Femto Bolt 相机"
piper_require_no_conda
piper_setup_ros

if [[ ! -f "${SCRIPT_DIR}/src/OrbbecSDK_ROS2/orbbec_camera/package.xml" ]]; then
    piper_error "Orbbec 子模块未初始化"
    piper_fix "git submodule update --init --recursive"
    exit 1
fi

piper_source_workspace "$SCRIPT_DIR"
if ! piper_has_ros_package orbbec_camera; then
    piper_error "工作空间中没有已构建的 orbbec_camera"
    piper_fix "bash scripts/build_workspace.sh --install-deps -- --packages-up-to orbbec_camera"
    exit 1
fi

piper_ok "相机驱动已找到"
piper_info "默认彩色话题: /camera/color/image_raw"
piper_info "默认相机内参: /camera/color/camera_info"
piper_info "默认深度话题: /camera/depth/image_raw"
piper_info "启动后可用 ros2 run orbbec_camera list_devices_node 检查设备"
piper_info "正在启动；Ctrl+C 停止..."

exec ros2 launch orbbec_camera femto_bolt.launch.py "$@"
