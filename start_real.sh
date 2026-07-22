#!/bin/bash
# Piper 真机环境设置脚本
# 用法: source ./start_real.sh
# 注意: 必须用 source 命令运行，否则环境不会在当前 shell 生效

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  Piper 真机环境设置"
echo "=========================================="

# 检查是否已在 conda 环境中
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo ">>> 已在 conda 环境: $CONDA_DEFAULT_ENV"
else
    # 尝试激活 piper_sdk 环境
    source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || true
    if conda activate piper_sdk 2>/dev/null; then
        echo ">>> 已激活: piper_sdk"
    else
        echo ">>> [警告] 未找到 piper_sdk 环境，请手动激活: conda activate piper_sdk"
    fi
fi

# Source ROS 2 环境 (fishros_humble)
source /opt/ros/humble/setup.bash
echo ">>> 已 source ROS 2 Humble (fishros_humble)"

# Source 工作空间
source "$SCRIPT_DIR/install/setup.bash"
echo ">>> 已 source PiperSim 工作空间"

echo ""
echo "环境已设置! 运行以下命令启动真机:"
echo ""
echo "  # 1. 先激活 CAN 总线(如需要)"
echo "  bash src/piper_control/scripts/can_activate.sh can0 1000000"
echo ""
echo "  # 2. 启动真机"
echo "  ros2 launch piper_moveit_config demo.launch.xml real_hardware:=true"
echo ""