#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=scripts/lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

install_deps=false
colcon_args=()

usage() {
    cat <<'EOF'
用法:
  bash scripts/build_workspace.sh [--install-deps] [-- COLCON参数...]

示例:
  bash scripts/build_workspace.sh --install-deps
  bash scripts/build_workspace.sh -- --packages-select piper_description piper_control
EOF
}

while (($#)); do
    case "$1" in
        --install-deps)
            install_deps=true
            shift
            ;;
        --)
            shift
            colcon_args=("$@")
            break
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            piper_error "未知参数: $1"
            usage >&2
            exit 2
            ;;
    esac
done

trap 'piper_error "构建在第 ${LINENO} 行失败（退出码: $?）"' ERR

piper_header "PiperSim 工作空间构建"
piper_info "项目目录: ${PROJECT_ROOT}"

piper_require_no_conda
piper_setup_ros

if [[ ! -f "${PROJECT_ROOT}/src/OrbbecSDK_ROS2/orbbec_camera/package.xml" ]]; then
    piper_warn "Orbbec 子模块未初始化；核心包可构建，相机驱动不会构建"
    piper_fix "git submodule update --init --recursive"
fi

other_distro="humble"
[[ "${PIPERSIM_ACTIVE_ROS_DISTRO}" == "humble" ]] && other_distro="jazzy"
if [[ -d "${PROJECT_ROOT}/build" ]] &&
   grep -Rqs --include='CMakeCache.txt' "/opt/ros/${other_distro}" "${PROJECT_ROOT}/build"; then
    piper_error "build/ 中检测到 ROS ${other_distro} 的缓存，不能与 ${PIPERSIM_ACTIVE_ROS_DISTRO} 混用"
    piper_fix "确认无需保留后，删除 build/ install/ log/，再重新构建"
    exit 1
fi

if "$install_deps"; then
    command -v rosdep >/dev/null 2>&1 || {
        piper_error "缺少 rosdep"
        piper_fix "sudo apt install python3-rosdep"
        exit 1
    }
    piper_info "更新 rosdep 索引并安装清单依赖..."
    rosdep update --rosdistro "${PIPERSIM_ACTIVE_ROS_DISTRO}"
    rosdep install \
        --from-paths "${PROJECT_ROOT}/src" \
        --ignore-src \
        --rosdistro "${PIPERSIM_ACTIVE_ROS_DISTRO}" \
        -r -y
fi

command -v colcon >/dev/null 2>&1 || {
    piper_error "缺少 colcon"
    piper_fix "sudo apt install python3-colcon-common-extensions"
    exit 1
}

# Prevent ~/.local Python wheels from shadowing ROS/apt packages during builds.
export PYTHONNOUSERSITE=1

piper_info "开始 colcon build（ROS ${PIPERSIM_ACTIVE_ROS_DISTRO} / Python $(piper_python_version)）..."
cd "${PROJECT_ROOT}"
colcon build --symlink-install "${colcon_args[@]}"

piper_ok "构建完成"
piper_info "让当前终端加载新构建:"
piper_command "source \"${PROJECT_ROOT}/install/setup.bash\""
