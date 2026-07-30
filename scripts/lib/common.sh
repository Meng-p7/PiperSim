#!/usr/bin/env bash

# Shared terminal and environment helpers for PiperSim shell scripts.
# This file is sourced; do not enable shell options here.

# ROS binary packages and python3-opencv must use the distribution Python
# packages. Keep user-installed wheels in ~/.local from shadowing them for every
# project entry point, not only while colcon is building.
export PYTHONNOUSERSITE=1

if [[ -t 1 && "${TERM:-dumb}" != "dumb" && -z "${NO_COLOR:-}" ]]; then
    PIPER_CLR_BLUE=$'\033[34m'
    PIPER_CLR_GREEN=$'\033[32m'
    PIPER_CLR_YELLOW=$'\033[33m'
    PIPER_CLR_RED=$'\033[31m'
    PIPER_CLR_BOLD=$'\033[1m'
    PIPER_CLR_RESET=$'\033[0m'
else
    PIPER_CLR_BLUE=""
    PIPER_CLR_GREEN=""
    PIPER_CLR_YELLOW=""
    PIPER_CLR_RED=""
    PIPER_CLR_BOLD=""
    PIPER_CLR_RESET=""
fi

piper_header() {
    printf '\n%s%s%s\n' "${PIPER_CLR_BOLD}" "$1" "${PIPER_CLR_RESET}"
    printf '%s\n' "────────────────────────────────────────────────────────"
}

piper_info() {
    printf '%s[INFO]%s %s\n' "${PIPER_CLR_BLUE}" "${PIPER_CLR_RESET}" "$*"
}

piper_ok() {
    printf '%s[ OK ]%s %s\n' "${PIPER_CLR_GREEN}" "${PIPER_CLR_RESET}" "$*"
}

piper_warn() {
    printf '%s[WARN]%s %s\n' "${PIPER_CLR_YELLOW}" "${PIPER_CLR_RESET}" "$*" >&2
}

piper_error() {
    printf '%s[FAIL]%s %s\n' "${PIPER_CLR_RED}" "${PIPER_CLR_RESET}" "$*" >&2
}

piper_fix() {
    printf '       修复: %s\n' "$*" >&2
}

piper_command() {
    printf '       %s$%s %s\n' "${PIPER_CLR_GREEN}" "${PIPER_CLR_RESET}" "$*"
}

piper_require_no_conda() {
    if [[ -n "${CONDA_PREFIX:-}" ]]; then
        piper_error "检测到 Conda 环境: ${CONDA_DEFAULT_ENV:-${CONDA_PREFIX}}"
        piper_fix "先执行 conda deactivate；ROS 二进制包必须使用系统 Python"
        return 1
    fi
}

piper_source_setup_file() {
    local setup_file="$1"
    local setup_status=0
    local restore_nounset=false

    # ROS-generated setup files are not guaranteed to be safe under `set -u`.
    # Keep the caller's setting, but disable nounset only while sourcing them.
    if [[ $- == *u* ]]; then
        restore_nounset=true
        set +u
    fi

    # shellcheck disable=SC1090
    source "$setup_file" || setup_status=$?

    if "$restore_nounset"; then
        set -u
    fi
    return "$setup_status"
}

piper_setup_ros() {
    local requested="${PIPERSIM_ROS_DISTRO:-}"
    local os_version=""
    local candidate=""
    local -a candidates=()

    if [[ -r /etc/os-release ]]; then
        os_version="$(
            # shellcheck disable=SC1091
            source /etc/os-release
            printf '%s' "${VERSION_ID:-}"
        )"
    fi

    if [[ -n "$requested" ]]; then
        case "$requested" in
            jazzy|humble) candidates+=("$requested") ;;
            *)
                piper_error "不支持 PIPERSIM_ROS_DISTRO=${requested}"
                piper_fix "仅可显式选择 jazzy 或 humble"
                return 1
                ;;
        esac
    elif [[ -n "${ROS_DISTRO:-}" ]]; then
        case "${ROS_DISTRO}" in
            jazzy|humble) candidates+=("${ROS_DISTRO}") ;;
            *)
                piper_error "当前 shell 已加载不受支持的 ROS ${ROS_DISTRO}"
                piper_fix "打开新终端并加载 Jazzy 或 Humble"
                return 1
                ;;
        esac
    fi
    if [[ -z "$requested" ]]; then
        case "$os_version" in
            24.04) candidates+=("jazzy") ;;
            22.04) candidates+=("humble") ;;
        esac
        candidates+=("jazzy" "humble")
    fi

    for candidate in "${candidates[@]}"; do
        [[ -n "$candidate" ]] || continue
        if [[ -r "/opt/ros/${candidate}/setup.bash" ]]; then
            if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "$candidate" ]]; then
                piper_error "当前 shell 已加载 ROS ${ROS_DISTRO}，不能再叠加 ROS ${candidate}"
                piper_fix "打开一个新终端，或 unset ROS_DISTRO 后重新运行"
                return 1
            fi
            piper_source_setup_file \
                "/opt/ros/${candidate}/setup.bash" || return 1
            export PIPERSIM_ACTIVE_ROS_DISTRO="$candidate"
            piper_ok "ROS 2 ${candidate}: /opt/ros/${candidate}"
            return 0
        fi
    done

    if [[ -n "$requested" ]]; then
        piper_error "显式请求的 ROS 2 ${requested} 未安装"
        piper_fix "安装 /opt/ros/${requested}，或取消 PIPERSIM_ROS_DISTRO"
    else
        piper_error "未找到受支持的 ROS 2 环境（Jazzy/Humble）"
        piper_fix "Ubuntu 24.04 安装 ROS 2 Jazzy，或按 README 使用 Docker"
    fi
    return 1
}

piper_source_workspace() {
    local project_root="$1"
    local setup_file="${project_root}/install/setup.bash"

    if [[ ! -r "$setup_file" ]]; then
        piper_error "工作空间尚未编译: ${setup_file}"
        piper_fix "cd \"${project_root}\" && bash scripts/build_workspace.sh --install-deps"
        return 1
    fi

    piper_source_setup_file "$setup_file" || return 1
    piper_ok "PiperSim 工作空间: ${project_root}/install"
}

piper_has_ros_package() {
    command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "$1" >/dev/null 2>&1
}

piper_source_mujoco_overlay() {
    local configured_overlay="${PIPERSIM_MUJOCO_OVERLAY:-}"
    local overlay_setup=""
    local overlay_install=""
    local overlay_workspace=""
    local cmake_cache=""
    local built_distro=""
    local distro=""

    if piper_has_ros_package mujoco_ros2_control; then
        if [[ -n "${PIPERSIM_MUJOCO_OVERLAY_ACTIVE:-}" &&
              ! -r "${PIPERSIM_MUJOCO_OVERLAY_ACTIVE}/setup.bash" ]]; then
            unset PIPERSIM_MUJOCO_OVERLAY_ACTIVE
        fi
        return 0
    fi

    unset PIPERSIM_MUJOCO_OVERLAY_ACTIVE

    if [[ -n "$configured_overlay" ]]; then
        if [[ -d "$configured_overlay" ]]; then
            overlay_setup="${configured_overlay%/}/setup.bash"
        else
            overlay_setup="$configured_overlay"
        fi
    elif [[ -n "${HOME:-}" ]]; then
        overlay_setup="${HOME}/mujoco_ros2_control_ws/install/setup.bash"
    fi

    if [[ -z "$overlay_setup" || ! -r "$overlay_setup" ]]; then
        if [[ -n "$configured_overlay" ]]; then
            piper_warn "PIPERSIM_MUJOCO_OVERLAY 不可读取: ${configured_overlay}"
        fi
        return 1
    fi

    overlay_install="$(cd "$(dirname "$overlay_setup")" && pwd)"
    overlay_workspace="$(cd "${overlay_install}/.." && pwd)"
    cmake_cache="${overlay_workspace}/build/mujoco_ros2_control/CMakeCache.txt"

    # A C++ ROS overlay is ABI-specific. Refuse to source an overlay built
    # against the other supported ROS distribution.
    if [[ -r "$cmake_cache" ]]; then
        for distro in humble jazzy; do
            if grep -Fq "/opt/ros/${distro}" "$cmake_cache"; then
                built_distro="$distro"
                break
            fi
        done
    fi
    if [[ -n "$built_distro" &&
          "$built_distro" != "${PIPERSIM_ACTIVE_ROS_DISTRO:-}" ]]; then
        piper_warn \
            "跳过 MuJoCo overlay：它为 ROS ${built_distro} 构建，当前为 ${PIPERSIM_ACTIVE_ROS_DISTRO:-unknown}"
        return 1
    fi

    piper_source_setup_file "$overlay_setup" || return 1
    if ! piper_has_ros_package mujoco_ros2_control; then
        piper_warn "MuJoCo overlay 已加载，但其中没有 mujoco_ros2_control: ${overlay_install}"
        return 1
    fi

    export PIPERSIM_MUJOCO_OVERLAY_ACTIVE="$overlay_install"
    return 0
}

piper_python_version() {
    python3 -c 'import platform; print(platform.python_version())' 2>/dev/null || printf 'unknown'
}
