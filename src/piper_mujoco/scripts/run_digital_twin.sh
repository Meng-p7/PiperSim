#!/usr/bin/env bash

set -Eeuo pipefail

resolved_script="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(cd "$(dirname "$resolved_script")" && pwd)"
twin_script="${script_dir}/digital_twin_sync_realtime.py"
check_only=false

if [[ "${1:-}" == "--check" ]]; then
    check_only=true
    shift
fi

if [[ ! -r "$twin_script" ]]; then
    printf '[FAIL] 找不到 Twin 同步脚本: %s\n' "$twin_script" >&2
    exit 1
fi

declare -a candidates=()
if [[ -n "${PIPERSIM_MUJOCO_PYTHON:-}" ]]; then
    candidates+=("${PIPERSIM_MUJOCO_PYTHON}")
fi

search_dir="$script_dir"
while [[ "$search_dir" != "/" ]]; do
    candidates+=("${search_dir}/.venv-mujoco/bin/python")
    search_dir="$(dirname "$search_dir")"
done
candidates+=("/opt/pipersim-venv/bin/python")
if command -v python3 >/dev/null 2>&1; then
    candidates+=("$(command -v python3)")
fi

for python_executable in "${candidates[@]}"; do
    [[ -x "$python_executable" ]] || continue
    if PYTHONNOUSERSITE=1 "$python_executable" -c \
        'import mujoco, rclpy' >/dev/null 2>&1; then
        if "$check_only"; then
            PYTHONNOUSERSITE=1 "$python_executable" - <<'PY'
import mujoco
import rclpy
print(f"[ OK ] Python: {__import__('sys').executable}")
print(f"[ OK ] MuJoCo: {mujoco.__version__}")
print(f"[ OK ] rclpy: {rclpy.__file__}")
PY
            exit 0
        fi
        export PYTHONNOUSERSITE=1
        exec "$python_executable" "$twin_script" "$@"
    fi
done

printf '[FAIL] Twin 找不到同时提供 mujoco 与 rclpy 的 Python\n' >&2
printf '       修复: 按 README 创建 .venv-mujoco，或设置 PIPERSIM_MUJOCO_PYTHON\n' >&2
exit 1
