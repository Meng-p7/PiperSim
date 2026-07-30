#!/usr/bin/env bash

# Source this file to configure the current shell for Mock/Sim development.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/scripts/lib/common.sh"

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    piper_error "start_sim.sh 必须用 source 运行，否则环境不会保留"
    piper_fix "source \"${SCRIPT_DIR}/start_sim.sh\""
    exit 2
fi

piper_header "PiperSim 仿真环境"
piper_require_no_conda || return 1
piper_setup_ros || return 1
piper_source_mujoco_overlay || true
piper_source_workspace "$SCRIPT_DIR" || return 1

piper_info "ROS: ${PIPERSIM_ACTIVE_ROS_DISTRO} | Python: $(piper_python_version)"
if piper_has_ros_package mujoco_ros2_control; then
    if [[ -n "${PIPERSIM_MUJOCO_OVERLAY_ACTIVE:-}" ]]; then
        piper_ok "MuJoCo ros2_control overlay: ${PIPERSIM_MUJOCO_OVERLAY_ACTIVE}"
    else
        piper_ok "MuJoCo ros2_control 插件可用"
    fi
else
    piper_warn "缺少 mujoco_ros2_control：Mock 可用，Sim 暂不可用"
    piper_fix "sudo apt install ros-${PIPERSIM_ACTIVE_ROS_DISTRO}-mujoco-ros2-control"
    piper_fix "已有源码工作区时：export PIPERSIM_MUJOCO_OVERLAY=/path/to/install"
fi

mujoco_python="python3"
if [[ -x "${SCRIPT_DIR}/.venv-mujoco/bin/python" ]]; then
    mujoco_python="${SCRIPT_DIR}/.venv-mujoco/bin/python"
fi

mujoco_version="$(
    "$mujoco_python" -c 'import mujoco; print(mujoco.__version__)' 2>/dev/null
)" || mujoco_version=""
if [[ -n "$mujoco_version" ]]; then
    export PIPERSIM_MUJOCO_PYTHON="$mujoco_python"
    piper_ok "Python MuJoCo ${mujoco_version}: ${mujoco_python}"
else
    piper_warn "缺少 Python mujoco：Twin/独立仿真暂不可用"
    piper_fix "按 README 创建 .venv-mujoco，并用该环境安装 mujoco==3.4.0"
fi

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    piper_warn "未检测到图形会话；完全无窗口需同时使用 rviz:=false headless:=true"
fi

piper_header "可用命令"
piper_info "Mock（无需 MuJoCo）"
piper_command "ros2 launch piper_moveit_config demo.launch.py mode:=mock"
piper_info "MuJoCo + MoveIt"
piper_command "ros2 launch piper_moveit_config demo.launch.py mode:=sim"
piper_info "MuJoCo 无窗口"
piper_command "ros2 launch piper_moveit_config demo.launch.py mode:=sim rviz:=false headless:=true"
piper_info "独立 MuJoCo GUI"
piper_command "\"${mujoco_python}\" \"${SCRIPT_DIR}/src/piper_mujoco/scripts/standalone_mujoco.py\""
