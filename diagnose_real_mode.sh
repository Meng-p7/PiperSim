#!/bin/bash
# Real模式快速诊断脚本

echo "=== Real Mode Quick Diagnosis ==="
echo ""
echo "1. 启动Real模式（后台）..."
source ~/PiperSim/install/setup.bash
ros2 launch piper_moveit_config demo.launch.py mode:=real &
LAUNCH_PID=$!

echo "等待10秒启动..."
sleep 10

echo ""
echo "=== 2. 检查控制器状态 ==="
ros2 control list_controllers

echo ""
echo "=== 3. 检查Action Servers ==="
ros2 action list | grep -E "(follow_joint_trajectory|gripper)"

echo ""
echo "=== 4. 检查Joint States话题 ==="
timeout 2 ros2 topic hz /joint_states 2>&1 | head -5

echo ""
echo "=== 5. 停止launch ==="
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null

echo ""
echo "=== 诊断完成 ==="