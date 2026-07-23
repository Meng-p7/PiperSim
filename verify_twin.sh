#!/bin/bash
# 完整的Twin模式验证脚本

echo "========================================"
echo "  Twin模式完整验证"
echo "========================================"

echo ""
echo "步骤1: 检查CAN状态"
if ip link show can0 &>/dev/null; then
    echo "✓ CAN0已配置"
else
    echo "✗ CAN0未配置，请先运行: source ~/PiperSim/start_real.sh"
    exit 1
fi

echo ""
echo "步骤2: Source环境"
source /opt/ros/humble/setup.bash || exit 1
source ~/PiperSim/install/setup.bash || exit 1

echo ""
echo "步骤3: 启动Twin模式（后台）"
ros2 launch piper_moveit_config demo.launch.py mode:=twin &
TWIN_PID=$!
sleep 10

echo ""
echo "步骤4: 检查节点"
NODES=$(ros2 node list)
echo "节点列表:"
echo "$NODES" | sed 's/^/  /'

echo ""
echo "步骤5: 检查话题"
TOPICS=$(ros2 topic list)
echo "话题列表:"
echo "$TOPICS" | sed 's/^/  /'

echo ""
echo "步骤6: 等待真机数据（请进入示教模式并拖动机械臂）"
echo "等待/joint_states数据..."
timeout 10 ros2 topic echo /joint_states --once > /tmp/joint_states.txt 2>&1

if [ -s /tmp/joint_states.txt ]; then
    echo "✓ 收到真机数据"
    head -20 /tmp/joint_states.txt
else
    echo "✗ 未收到真机数据"
    echo ""
    echo "可能的原因："
    echo "1. 真机未启动（请确认CAN已激活）"
    echo "2. 未进入示教模式"
    echo "3. 控制器未正确启动"
fi

echo ""
echo "步骤7: 启动同步脚本"
echo "按Ctrl+C停止..."
python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_realtime.py

# 清理
kill $TWIN_PID 2>/dev/null