#!/bin/bash
# 手动启动Twin模式（分步执行）

echo "========================================"
echo "  手动启动Twin模式"
echo "========================================"

# 检查环境
source /opt/ros/humble/setup.bash || exit 1
source ~/PiperSim/install/setup.bash || exit 1

echo ""
echo "步骤1: 启动Twin模式（后台运行）"
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source ~/PiperSim/install/setup.bash && ros2 launch piper_moveit_config demo.launch.py mode:=twin; exec bash" &
sleep 12

echo ""
echo "步骤2: 检查节点启动状态"
for i in {1..5}; do
    echo "检查节点 (尝试 $i/5)..."
    NODES=$(ros2 node list 2>&1)
    echo "$NODES" | grep -q "mujoco" && echo "✓ MuJoCo节点已启动" && break
    sleep 2
done

echo ""
echo "步骤3: 检查话题"
TOPICS=$(ros2 topic list 2>&1)
echo "当前话题:"
echo "$TOPICS" | grep -E "(joint|mujoco)"

echo ""
echo "步骤4: 启动同步脚本"
echo "现在请在新终端运行:"
echo "  python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_realtime.py"
echo ""
echo "或者按回车键自动启动..."
read -r

python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_realtime.py