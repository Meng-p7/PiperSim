#!/bin/bash
# 安装 mujoco_ros2_control（Jazzy版本，直接apt安装）
# 用法: bash docker/install_mujoco_jazzy.sh

set -e

echo "========================================"
echo "安装 mujoco_ros2_control（Jazzy）"
echo "========================================"

# 检查是否在容器内
if [ ! -f "/.dockerenv" ]; then
    echo "错误: 此脚本必须在 Docker 容器内运行"
    echo "请先进入容器: docker exec -it pipersim bash"
    exit 1
fi

echo ">>> 使用 apt 安装（Jazzy版本无需编译！）"
apt-get update
apt-get install -y \
    ros-jazzy-mujoco-ros2-control \
    ros-jazzy-mujoco-ros2-control-demos

echo ""
echo "✅ mujoco_ros2_control 安装完成！"
echo ""
echo "优势："
echo "  - 安装速度：从5-10分钟编译 → 几秒安装"
echo "  - 无需手动编译 mujoco_vendor"
echo "  - 自动包含在 ROS 2 Jazzy 环境中"
echo ""
echo "验证安装："
echo "  dpkg -l | grep mujoco-ros2-control"
echo ""
echo "测试运行："
echo "  ros2 launch mujoco_ros2_control_demos demo.launch.py"