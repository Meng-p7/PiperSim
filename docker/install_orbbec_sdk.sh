#!/bin/bash
# 安装 OrbbecSDK ROS2 驱动（Femto Bolt相机）
# 用法: bash docker/install_orbbec_sdk.sh

set -e

echo "========================================"
echo "安装 OrbbecSDK ROS2 Wrapper"
echo "========================================"

# 检查是否在容器内
if [ ! -f "/.dockerenv" ]; then
    echo "错误: 此脚本必须在 Docker 容器内运行"
    echo "请先进入容器: docker exec -it pipersim bash"
    exit 1
fi

# 克隆源码
cd /workspace/src
if [ -d "OrbbecSDK_ROS2" ]; then
    echo ">>> OrbbecSDK_ROS2 已存在，跳过克隆"
else
    echo ">>> 克隆 OrbbecSDK_ROS2..."
    git clone https://github.com/orbbec/OrbbecSDK_ROS2.git -b v2-main
fi

# 安装 udev 规则（USB设备访问权限）
echo ">>> 安装 udev 规则..."
cd /workspace/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger

# 安装 ROS 依赖
echo ">>> 安装 ROS 依赖..."
cd /workspace
source /opt/ros/jazzy/setup.bash
rosdep update || true
rosdep install -r --from-paths src/OrbbecSDK_ROS2 --ignore-src --rosdistro jazzy -y || true

# 编译 Orbbec 包
echo ">>> 编译 Orbbec SDK..."
cd /workspace
colcon build --symlink-install \
    --packages-select orbbec_camera orbbec_camera_msgs orbbec_description \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "✅ Orbbec SDK 安装完成！"
echo ""
echo "使用方法："
echo "  # 在容器内启动 Femto Bolt 相机"
echo "  source /workspace/install/setup.bash"
echo "  ros2 launch orbbec_camera femto_bolt.launch.py"
echo ""
echo "  # 查看相机话题"
echo "  ros2 topic list | grep camera"