#!/bin/bash
# 诊断脚本：检查关节状态和规划问题

echo "=========================================="
echo "   Piper 机械臂诊断脚本"
echo "=========================================="

# 清理残留进程
pkill -f "ros2 launch" 2>/dev/null
pkill -f "move_group" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
sleep 2

# Source 环境
cd /home/dream/PiperSim
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动 mock_bringup
echo "[1] 启动 mock_bringup..."
ros2 launch piper_bringup mock_bringup.launch.py &>/dev/null &
LAUNCH_PID=$!
sleep 10

echo "[2] 检查 joint_states 发布者数量..."
PUB_COUNT=$(ros2 topic info /joint_states 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
echo "    发布者数量: $PUB_COUNT"
if [ "$PUB_COUNT" -gt 1 ]; then
    echo "    [警告] 有多个发布者，可能导致状态冲突！"
fi

echo "[3] 获取关节状态..."
ros2 topic echo /joint_states --once 2>/dev/null

echo ""
echo "[4] 检查 TF 树..."
ros2 run tf2_tools view_tf_tree 2>/dev/null || true

echo ""
echo "[5] 测试运动规划服务..."
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetKinematicPath \
    "{group_name: 'manipulator', start_state: {joint_state: {name: ['joint1','joint2','joint3','joint4','joint5','joint6'], position: [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]}}, goal_constraints: [{joint_constraints: [{joint_name: 'joint1', position: 0.3}]}]}" \
    2>&1 | head -20

echo ""
echo "[6] 清理..."
kill $LAUNCH_PID 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null

echo "=========================================="
echo "   诊断完成"
echo "=========================================="