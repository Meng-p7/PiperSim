#!/bin/bash
# Piper 仿真环境设置脚本
# 用法: source ./start_sim.sh
# 注意: 必须用 source 命令运行，否则环境不会在当前 shell 生效

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  Piper 仿真环境设置"
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

# 自动加载 mujoco_ros2_control（如果已编译）
MUJOCO_WS=~/mujoco_ros2_control_ws
if [ -f "$MUJOCO_WS/install/setup.bash" ]; then
    source "$MUJOCO_WS/install/setup.bash"
    echo ">>> 已 source mujoco_ros2_control"
fi

echo ""
echo "环境已设置! 运行以下命令启动仿真:"
echo ""
echo "  # Mock 模式（纯软件仿真，推荐开发调试）"
echo "  ros2 launch piper_moveit_config demo.launch.py mode:=mock"
echo ""
echo "  # Sim 模式（MuJoCo物理仿真 + MoveIt控制）"
echo "  ros2 launch piper_moveit_config demo.launch.py mode:=sim"
echo ""
echo "  # 独立MuJoCo仿真（仅GUI手动控制，无ROS2干扰）"
echo "  python3 ~/PiperSim/src/piper_mujoco/scripts/standalone_mujoco.py"
echo ""