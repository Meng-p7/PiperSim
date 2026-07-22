#!/bin/bash
# Piper 环境设置脚本
# 用法：source ./setup_env.sh
# 此脚本用于在当前 shell 中设置 ROS 2 环境

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "设置 PiperSim 环境..."

# 检查是否已在 conda 环境中
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    # 尝试激活 conda 环境
    source ~/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source ~/anaconda3/etc/profile.d/conda.sh 2>/dev/null || true
    if conda activate fishros_humble 2>/dev/null; then
        echo ">>> 已激活 conda: fishros_humble"
    elif conda activate piper_sdk 2>/dev/null; then
        echo ">>> 已激活 conda: piper_sdk"
    else
        echo ">>> 未激活 conda 环境，使用系统环境"
    fi
else
    echo ">>> 已在 conda 环境: $CONDA_DEFAULT_ENV"
fi

# Source ROS 2 环境
source /opt/ros/humble/setup.bash
echo ">>> 已 source ROS 2 Humble"

# Source 工作空间
source "$SCRIPT_DIR/install/setup.bash"
echo ">>> 已 source PiperSim 工作空间"

echo ""
echo "环境已设置完成！你现在可以运行："
echo "  ros2 launch piper_moveit_config demo.launch.xml                      # Mock 仿真"
echo "  ros2 launch piper_moveit_config demo.launch.xml sim_gazebo:=true     # Gazebo 仿真"
echo "  ros2 launch piper_moveit_config demo.launch.xml real_hardware:=true  # 真机"