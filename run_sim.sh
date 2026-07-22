#!/bin/bash
# 完整的 Piper 仿真启动脚本
# 用法: source ./run_sim.sh

echo "=========================================="
echo "   Piper 机械臂仿真环境设置"
echo "=========================================="

# 清理残留进程
echo "[1/5] 清理残留进程..."
pkill -f "ros2 launch" 2>/dev/null
pkill -f "move_group" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "ros2_control" 2>/dev/null
sleep 2

# 激活 conda 环境
echo "[2/5] 设置环境..."
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "当前已在 conda 环境: $CONDA_DEFAULT_ENV"
else
    for env in fishros_humble piper_sdk; do
        if conda env list | grep -q "^$env "; then
            echo "激活 conda 环境: $env"
            eval "$(conda shell.bash hook)"
            conda activate $env 2>/dev/null && break
        fi
    done
fi

# Source ROS 2 环境
echo "[3/5] Source ROS 2 环境..."
source /opt/ros/humble/setup.bash

# Source 工作空间
echo "[4/5] Source 工作空间..."
cd /home/dream/PiperSim
source install/setup.bash

echo "[5/5] 环境设置完成!"
echo ""
echo "现在可以运行以下命令:"
echo "  ros2 launch piper_moveit_config demo.launch.xml           # Mock 仿真"
echo "  ros2 launch piper_moveit_config demo.launch.xml sim_gazebo:=true  # Gazebo 仿真"
echo ""
echo "=========================================="