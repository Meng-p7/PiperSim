#!/bin/bash
# 启动 Orbbec Femto Bolt 相机节点
# 用法: source ~/PiperSim/start_orbbec_camera.sh

echo "========================================"
echo "启动 Orbbec Femto Bolt 相机"
echo "========================================"

# 检查是否已安装 Orbbec SDK
if [ ! -d "$HOME/PiperSim/src/OrbbecSDK_ROS2" ]; then
    echo "错误: OrbbecSDK_ROS2 未安装"
    echo "请先运行: bash docker/install_orbbec_sdk.sh"
    return 1
fi

# 激活环境
source /opt/ros/jazzy/setup.bash
source ~/PiperSim/install/setup.bash

# 启动 Femto Bolt 相机
echo ">>> 启动 Femto Bolt 相机节点..."
ros2 launch orbbec_camera femto_bolt.launch.py

echo ""
echo "相机话题:"
echo "  - /camera/camera/color/image_raw"
echo "  - /camera/camera/color/camera_info"
echo "  - /camera/camera/depth/image_raw"
echo "  - /camera/camera/depth/camera_info"