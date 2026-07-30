#!/usr/bin/env bash

# Source this file to configure the current shell for real-hardware work.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/scripts/lib/common.sh"

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    piper_error "start_real.sh 必须用 source 运行，否则环境不会保留"
    piper_fix "source \"${SCRIPT_DIR}/start_real.sh\""
    exit 2
fi

piper_header "PiperSim 真机环境"
piper_require_no_conda || return 1
piper_setup_ros || return 1
piper_source_workspace "$SCRIPT_DIR" || return 1

if ! piper_has_ros_package piper_control; then
    piper_error "未找到 piper_control 硬件插件"
    piper_fix "bash \"${SCRIPT_DIR}/scripts/build_workspace.sh\""
    return 1
fi
piper_ok "真机硬件插件可用"

mujoco_python=""
mujoco_version=""
for candidate in \
    "${PIPERSIM_MUJOCO_PYTHON:-}" \
    "${SCRIPT_DIR}/.venv-mujoco/bin/python" \
    "/opt/pipersim-venv/bin/python"; do
    [[ -n "$candidate" && -x "$candidate" ]] || continue
    candidate_version="$(
        "$candidate" -c \
            'import mujoco, rclpy; print(mujoco.__version__)' 2>/dev/null
    )" || candidate_version=""
    if [[ -n "$candidate_version" ]]; then
        mujoco_python="$candidate"
        mujoco_version="$candidate_version"
        break
    fi
done
if [[ -n "$mujoco_version" ]]; then
    export PIPERSIM_MUJOCO_PYTHON="$mujoco_python"
    piper_ok "Twin Python MuJoCo ${mujoco_version}: ${mujoco_python}"
else
    piper_warn "Twin 缺少可用的项目 MuJoCo Python；Real 模式不受影响"
    piper_fix "按 README 创建 .venv-mujoco 并安装 mujoco==3.4.0"
fi

can_report=""
if command -v ip >/dev/null 2>&1; then
    can_report="$(
        ip -brief link show type can 2>/dev/null || true
        ip -brief link show type vcan 2>/dev/null || true
    )"
fi
if [[ -n "$can_report" ]]; then
    if awk '$2 == "UP" {found=1} END {exit !found}' <<<"$can_report"; then
        piper_ok "检测到已启动的 CAN/vcan 接口: ${can_report//$'\n'/; }"
    else
        piper_warn "检测到 CAN 接口，但当前未启动: ${can_report//$'\n'/; }"
        piper_fix \
            "bash \"${SCRIPT_DIR}/src/piper_control/scripts/can_activate.sh\" can0 1000000"
    fi
else
    piper_warn "未检测到 CAN/vcan 接口；启动 ROS 前先配置适配器"
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    piper_warn "未检测到图形会话；Twin 的 MuJoCo viewer 无法启动，Real 模式不受影响"
fi

piper_header "安全提示"
piper_warn "真机启动会使能电机；必须清空工作区、降低速度并准备物理急停"
piper_warn "首次迁移到新主机时，先做 vcan/台架验证，不要直接带负载运行"

piper_header "推荐启动顺序"
piper_info "1. 配置 CAN（接口名与波特率可修改）"
piper_command "bash \"${SCRIPT_DIR}/src/piper_control/scripts/can_activate.sh\" can0 1000000"
piper_info "2. 真机 MoveIt"
piper_command "ros2 launch piper_moveit_config demo.launch.py mode:=real can:=can0"
piper_info "数字孪生（需要 Python mujoco）"
piper_command "ros2 launch piper_moveit_config demo.launch.py mode:=twin can:=can0"
piper_info "人工拖动标定（只读反馈，不发送运动命令）"
piper_command "ros2 launch piper_bringup real_bringup.launch.py can:=can0 calibration_mode:=true"
