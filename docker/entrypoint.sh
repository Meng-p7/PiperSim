#!/usr/bin/env bash

# This file is both the container entrypoint and the setup sourced by
# interactive `docker compose exec ... bash` shells.

_pipersim_container_setup() {
    local ros_setup="/opt/ros/jazzy/setup.bash"
    local venv_setup="/opt/pipersim-venv/bin/activate"
    local workspace_setup="/workspace/install/setup.bash"
    local build_dir

    if [[ ! -r "$ros_setup" ]]; then
        printf '[FAIL] 缺少 ROS 2 Jazzy: %s\n' "$ros_setup" >&2
        return 1
    fi

    for build_dir in /workspace/build /workspace/install /workspace/log; do
        if [[ ! -d "$build_dir" || ! -w "$build_dir" ]]; then
            printf '[FAIL] Jazzy 构建卷不可写: %s\n' "$build_dir" >&2
            printf '       请按 README 的“Docker 构建卷不可写”处理，勿直接删除有数据的卷\n' >&2
            return 1
        fi
    done

    # shellcheck disable=SC1090
    source "$ros_setup" || return 1
    # shellcheck disable=SC1090
    source "$venv_setup" || return 1

    if [[ -r "$workspace_setup" ]]; then
        # shellcheck disable=SC1090
        source "$workspace_setup" || return 1
        if [[ $- == *i* ]]; then
            printf '[ OK ] ROS 2 Jazzy + PiperSim workspace 已加载\n'
        fi
    elif [[ $- == *i* ]]; then
        printf '[WARN] 工作空间尚未构建\n'
        printf '       运行: bash scripts/build_workspace.sh --install-deps\n'
    fi
}

_pipersim_container_setup
setup_status=$?
unset -f _pipersim_container_setup

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    ((setup_status == 0)) || exit "$setup_status"
    exec "$@"
else
    return "$setup_status"
fi
