#!/bin/bash
# Docker容器入口点脚本

# Source ROS 2
source /opt/ros/humble/setup.bash

# 激活conda环境
source /opt/conda/etc/profile.d/conda.sh
conda activate piper_sdk

# 如果工作空间已编译，source它
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# 如果mujoco_ros2_control已安装，source它
# 注意：mujoco_ros2_control默认安装在 ~/mujoco_ros2_control_ws
MUJOCO_WS="/root/mujoco_ros2_control_ws"
if [ -f "$MUJOCO_WS/install/setup.bash" ]; then
    source $MUJOCO_WS/install/setup.bash
fi

# 执行用户命令
exec "$@"