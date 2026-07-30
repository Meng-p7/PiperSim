#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TARGET="${1:-all}"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

if (($# > 1)); then
    piper_error "参数过多: ${*:2}"
    printf '用法: bash scripts/doctor.sh [all|sim|real|camera|docker]\n' >&2
    exit 2
fi

case "$TARGET" in
    all|sim|real|camera|docker) ;;
    -h|--help)
        printf '用法: bash scripts/doctor.sh [all|sim|real|camera|docker]\n'
        exit 0
        ;;
    *)
        piper_error "未知检查目标: ${TARGET}"
        exit 2
        ;;
esac

failures=0
warnings=0

pass() {
    piper_ok "$*"
}

warn() {
    warnings=$((warnings + 1))
    piper_warn "$*"
}

fail() {
    failures=$((failures + 1))
    piper_error "$*"
}

has_command() {
    command -v "$1" >/dev/null 2>&1
}

piper_header "PiperSim 环境诊断: ${TARGET}"

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    pass "系统: ${PRETTY_NAME:-unknown} ($(uname -m))"
else
    warn "无法读取 /etc/os-release"
fi

if [[ -n "${CONDA_PREFIX:-}" ]]; then
    fail "Conda 正在生效: ${CONDA_DEFAULT_ENV:-${CONDA_PREFIX}}"
else
    pass "未检测到 Conda 污染"
fi

if piper_setup_ros >/dev/null 2>&1; then
    pass "ROS 2: ${PIPERSIM_ACTIVE_ROS_DISTRO} / Python $(piper_python_version)"
else
    if [[ "$TARGET" != "docker" ]]; then
        fail "未找到 ROS 2 Jazzy/Humble"
        piper_fix "Ubuntu 24.04 安装 Jazzy，Ubuntu 22.04 安装 Humble，或使用 Docker"
    else
        warn "宿主未安装 ROS（Docker 模式不要求）"
    fi
fi

if [[ "$TARGET" == "docker" ]]; then
    piper_info "Docker 使用独立构建卷；不检查宿主 build/install"
elif [[ -r "${PROJECT_ROOT}/install/setup.bash" ]]; then
    pass "工作空间已构建"
else
    warn "工作空间未构建；运行 bash scripts/build_workspace.sh --install-deps"
fi

if [[ -f "${PROJECT_ROOT}/src/OrbbecSDK_ROS2/orbbec_camera/package.xml" ]]; then
    pass "Orbbec 子模块已初始化"
elif [[ "$TARGET" == "all" || "$TARGET" == "camera" ]]; then
    fail "Orbbec 子模块为空；运行 git submodule update --init --recursive"
else
    warn "Orbbec 子模块未初始化（当前检查目标不依赖相机）"
fi

if [[ "$TARGET" == "all" || "$TARGET" == "sim" ]]; then
    piper_source_mujoco_overlay || true
    if piper_has_ros_package mujoco_ros2_control; then
        if [[ -n "${PIPERSIM_MUJOCO_OVERLAY_ACTIVE:-}" ]]; then
            pass "mujoco_ros2_control overlay: ${PIPERSIM_MUJOCO_OVERLAY_ACTIVE}"
        else
            pass "mujoco_ros2_control 可用"
        fi
    else
        warn "缺少 mujoco_ros2_control；Mock 仍可用，Sim 不可用"
    fi
    mujoco_python="python3"
    if [[ -x "${PROJECT_ROOT}/.venv-mujoco/bin/python" ]]; then
        mujoco_python="${PROJECT_ROOT}/.venv-mujoco/bin/python"
    fi
    mujoco_version="$(
        "$mujoco_python" -c 'import mujoco; print(mujoco.__version__)' 2>/dev/null
    )" || mujoco_version=""
    if [[ -n "$mujoco_version" ]]; then
        pass "Python MuJoCo ${mujoco_version}: ${mujoco_python}"
    else
        warn "缺少 Python mujoco；独立仿真和 Twin 不可用"
    fi
    display_part="${DISPLAY:-}"
    display_part="${display_part##*:}"
    display_number="${display_part%%.*}"
    if [[ "$display_number" =~ ^[0-9]+$ ]] &&
       [[ -S "/tmp/.X11-unix/X${display_number}" ]]; then
        pass "图形显示: DISPLAY=${DISPLAY}"
    else
        warn "未确认 X11 DISPLAY；RViz/MuJoCo GUI 可能无法打开"
    fi
fi

if [[ "$TARGET" == "all" || "$TARGET" == "real" ]]; then
    can_report=""
    if has_command ip; then
        can_report="$(
            ip -brief link show type can 2>/dev/null || true
            ip -brief link show type vcan 2>/dev/null || true
        )"
        if [[ -n "$can_report" ]]; then
            pass "检测到 CAN/vcan 接口: ${can_report//$'\n'/; }"
        else
            warn "未检测到 CAN/vcan 接口"
        fi
    else
        warn "无法读取 SocketCAN 接口（容器需用 --can 启动）"
    fi
fi

if [[ "$TARGET" == "all" || "$TARGET" == "camera" ]]; then
    if has_command lsusb && lsusb 2>/dev/null | grep -qi '2bc5'; then
        pass "检测到 Orbbec USB 设备"
    else
        warn "未检测到 Orbbec USB 设备（或缺少 lsusb）"
    fi
    opencv_report="$(
        python3 -c '
import cv2
assert hasattr(cv2, "aruco")
assert hasattr(cv2, "calibrateHandEye")
print(f"{cv2.__version__} ({cv2.__file__})")
' 2>/dev/null
    )" || opencv_report=""
    if [[ -n "$opencv_report" ]]; then
        pass "OpenCV 标定 API: ${opencv_report}"
    else
        fail "OpenCV 缺少 aruco/calibrateHandEye，或 cv2 无法导入"
        piper_fix "退出 Conda，设置 PYTHONNOUSERSITE=1，并用 apt 安装 python3-opencv"
    fi
fi

if [[ "$TARGET" == "all" || "$TARGET" == "docker" ]]; then
    if has_command docker; then
        pass "Docker CLI: $(docker --version 2>/dev/null)"
        if docker compose version >/dev/null 2>&1; then
            pass "Compose plugin: $(docker compose version --short 2>/dev/null)"
            compose_base="${PROJECT_ROOT}/docker/docker-compose.yml"
            compose_ok=true
            for override in gui gpu camera can; do
                if ! DISPLAY="${DISPLAY:-:0}" docker compose \
                    -f "$compose_base" \
                    -f "${PROJECT_ROOT}/docker/docker-compose.${override}.yml" \
                    config --quiet >/dev/null 2>&1; then
                    compose_ok=false
                    fail "Compose 配置解析失败: ${override}"
                fi
            done
            if "$compose_ok"; then
                if DISPLAY="${DISPLAY:-:0}" docker compose \
                    -f "$compose_base" \
                    -f "${PROJECT_ROOT}/docker/docker-compose.gui.yml" \
                    -f "${PROJECT_ROOT}/docker/docker-compose.gpu.yml" \
                    -f "${PROJECT_ROOT}/docker/docker-compose.camera.yml" \
                    -f "${PROJECT_ROOT}/docker/docker-compose.can.yml" \
                    config --quiet >/dev/null 2>&1; then
                    pass "基础、各能力及完整组合 Compose 配置有效"
                else
                    fail "Compose 完整组合解析失败"
                fi
            fi
        else
            fail "缺少 Docker Compose plugin"
        fi
        if docker info >/dev/null 2>&1; then
            pass "Docker daemon 可访问"
        else
            fail "Docker daemon 不可访问；检查服务与 docker 组"
        fi
    else
        fail "未安装 Docker"
    fi
fi

piper_header "诊断结果"
printf '失败: %d，警告: %d\n' "$failures" "$warnings"
if ((failures > 0)); then
    exit 1
fi
exit 0
