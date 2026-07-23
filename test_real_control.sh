#!/bin/bash
# 完整真机控制测试

echo "=== 真机控制完整测试 ==="
echo ""

echo "1. 启动Real模式..."
source ~/PiperSim/install/setup.bash
ros2 launch piper_moveit_config demo.launch.py mode:=real &
LAUNCH_PID=$!
sleep 12

echo ""
echo "2. 检查控制器状态..."
ros2 control list_controllers

echo ""
echo "3. 发送测试轨迹..."
python3 ~/PiperSim/test_real_trajectory.py

echo ""
echo "4. 检查执行结果..."
echo "如果真机移动了，说明控制成功！"
echo ""

echo "5. 停止launch..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null

echo "测试完成"