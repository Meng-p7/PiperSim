#!/bin/bash
# 从源码编译安装 mujoco_ros2_control
# 用法: bash install_mujoco_ros2_control_from_source.sh

set -e

echo "========================================"
echo "安装 mujoco_ros2_control (源码编译)"
echo "========================================"

# 创建工作空间
MUJOCO_WS=~/mujoco_ros2_control_ws
mkdir -p $MUJOCO_WS/src
cd $MUJOCO_WS/src

# 克隆源码（使用 main 分支）
echo ">>> 克隆 mujoco_ros2_control..."
git clone https://github.com/ros-controls/mujoco_ros2_control -b main

# 克隆依赖
echo ">>> 克隆 mujoco_vendor..."
git clone https://github.com/pal-robotics/mujoco_vendor -b master

# 安装依赖
echo ">>> 安装依赖..."
cd $MUJOCO_WS
source /opt/ros/humble/setup.bash
rosdep update || true
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y || true

# 编译（跳过可选插件）
echo ">>> 编译中..."
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-skip mujoco_ros2_control_plugins mujoco_ros2_control_tests

echo ""
echo "✅ 编译完成！"
echo ""
echo "注意: 已跳过可选包 mujoco_ros2_control_plugins 和 mujoco_ros2_control_tests"
echo ""
echo "使用方法："
echo "  source $MUJOCO_WS/install/setup.bash"
echo "  cd ~/PiperSim"
echo "  source install/setup.bash"
echo "  ros2 launch piper_moveit_config demo.launch.xml sim_mujoco:=true"
echo ""
echo "或者直接运行:"
echo "  source $MUJOCO_WS/install/setup.bash && source ~/PiperSim/start_sim.sh"
echo "  ros2 launch piper_moveit_config demo.launch.xml sim_mujoco:=true"