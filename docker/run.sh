#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck source=scripts/lib/common.sh
source "${PROJECT_ROOT}/scripts/lib/common.sh"

usage() {
    cat <<'EOF'
用法:
  ./docker/run.sh up [--gui] [--gpu] [--camera] [--can] [--hardware]
  ./docker/run.sh build
  ./docker/run.sh shell
  ./docker/run.sh logs
  ./docker/run.sh status
  ./docker/run.sh config [--gui] [--gpu] [--camera] [--can] [--hardware]
  ./docker/run.sh down

默认配置为 CPU/headless，不要求 NVIDIA，也不共享 GUI、USB 或宿主网络。
--gui       叠加 X11 GUI 配置
--gpu       叠加 NVIDIA GPU 配置
--camera    叠加 Orbbec USB 设备访问
--can       叠加 SocketCAN 所需的宿主网络（CAN 必须先在宿主配置）
--hardware  兼容别名，同时启用 --camera 与 --can
EOF
}

action="${1:-help}"
[[ $# -gt 0 ]] && shift
use_gui=false
use_gpu=false
use_camera=false
use_can=false

while (($#)); do
    case "$1" in
        --gui) use_gui=true ;;
        --gpu) use_gpu=true ;;
        --camera) use_camera=true ;;
        --can) use_can=true ;;
        --hardware)
            use_camera=true
            use_can=true
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
    shift
done

case "$action" in
    up|build|shell|logs|status|config|down) ;;
    help|-h|--help)
        usage
        exit 0
        ;;
    *)
        piper_error "未知操作: ${action}"
        usage >&2
        exit 2
        ;;
esac

if [[ "$action" != "up" && "$action" != "config" ]] &&
   { "$use_gui" || "$use_gpu" || "$use_camera" || "$use_can"; }; then
    piper_error "能力选项只适用于 up 或 config；不能用于 ${action}"
    exit 2
fi

if ((EUID == 0)) || [[ "$(id -g)" == "0" ]]; then
    piper_error "不要用 sudo/root 运行 docker/run.sh"
    piper_fix "按 README 配置 Docker 用户组并重新登录，再以普通用户运行"
    exit 1
fi

command -v docker >/dev/null 2>&1 || {
    piper_error "未安装 Docker Engine"
    piper_fix "按 README 的 Ubuntu 24.04 官方仓库步骤安装"
    exit 1
}
docker compose version >/dev/null 2>&1 || {
    piper_error "缺少 Docker Compose plugin"
    piper_fix "sudo apt install docker-compose-plugin"
    exit 1
}
docker info >/dev/null 2>&1 || {
    piper_error "Docker daemon 不可访问"
    piper_fix "sudo systemctl enable --now docker；然后检查 docker 组权限"
    exit 1
}

export PIPERSIM_UID
export PIPERSIM_GID
PIPERSIM_UID="$(id -u)"
PIPERSIM_GID="$(id -g)"

compose_files=(-f "${SCRIPT_DIR}/docker-compose.yml")
if "$use_gui"; then
    compose_files+=(-f "${SCRIPT_DIR}/docker-compose.gui.yml")
fi
if "$use_gpu"; then
    compose_files+=(-f "${SCRIPT_DIR}/docker-compose.gpu.yml")
fi
if "$use_camera"; then
    compose_files+=(-f "${SCRIPT_DIR}/docker-compose.camera.yml")
fi
if "$use_can"; then
    compose_files+=(-f "${SCRIPT_DIR}/docker-compose.can.yml")
fi
compose=(docker compose "${compose_files[@]}")

preflight() {
    local check_runtime="${1:-true}"

    piper_header "PiperSim Docker 预检"
    piper_ok "Docker: $(docker --version)"
    piper_ok "Compose: $(docker compose version --short)"
    piper_info "容器用户映射: UID=${PIPERSIM_UID}, GID=${PIPERSIM_GID}"

    if [[ ! -f "${PROJECT_ROOT}/src/OrbbecSDK_ROS2/orbbec_camera/package.xml" ]]; then
        piper_warn "Orbbec 子模块未初始化"
        piper_fix "git submodule update --init --recursive"
    fi
    if "$check_runtime" && "$use_gui"; then
        [[ -n "${DISPLAY:-}" ]] || {
            piper_error "--gui 需要宿主 DISPLAY"
            exit 1
        }
        [[ -d /tmp/.X11-unix ]] || {
            piper_error "--gui 需要宿主 /tmp/.X11-unix"
            exit 1
        }
        piper_ok "GUI DISPLAY=${DISPLAY}"
    fi
    if "$check_runtime" && "$use_gpu"; then
        command -v nvidia-smi >/dev/null 2>&1 || {
            piper_error "--gpu 需要宿主 NVIDIA 驱动与 Container Toolkit"
            exit 1
        }
        piper_ok "NVIDIA GPU 配置已启用"
    fi
    if "$check_runtime" && "$use_camera"; then
        [[ -d /dev/bus/usb ]] || {
            piper_error "--camera 需要宿主 /dev/bus/usb"
            exit 1
        }
        piper_warn "已启用 USB 设备访问；仅在相机任务中使用"
    fi
    if "$check_runtime" && "$use_can"; then
        piper_warn "已启用 host network 供 SocketCAN 使用；接口必须在宿主机配置"
        can_report=""
        if command -v ip >/dev/null 2>&1; then
            can_report="$(
                ip -brief link show type can 2>/dev/null || true
                ip -brief link show type vcan 2>/dev/null || true
            )"
        fi
        if [[ -z "$can_report" ]]; then
            piper_warn "宿主机当前未检测到 CAN/vcan 接口"
        fi
    fi
}

case "$action" in
    up)
        preflight true
        "${compose[@]}" up -d --build
        piper_ok "容器已启动"
        piper_command "./docker/run.sh shell"
        ;;
    build)
        preflight false
        "${compose[@]}" build
        ;;
    shell)
        if [[ -t 0 && -t 1 ]]; then
            "${compose[@]}" exec pipersim bash
        else
            "${compose[@]}" exec -T pipersim \
                bash -c 'source /entrypoint.sh && exec bash'
        fi
        ;;
    logs)
        "${compose[@]}" logs --tail=200 -f pipersim
        ;;
    status)
        "${compose[@]}" ps
        ;;
    config)
        "${compose[@]}" config
        ;;
    down)
        "${compose[@]}" down
        piper_ok "容器已停止；Jazzy 构建卷仍保留"
        ;;
esac
