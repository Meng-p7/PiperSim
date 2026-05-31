#!/bin/bash
# 清理所有可能占用 CAN 总线的进程
# 用法: bash clean_can.sh

echo "=== 清理 ROS 进程 ==="
pids=$(ps aux | grep -E "ros2|controller_manager|robot_state|rviz2|spawner|real_bringup|move_group|calibration" | grep -v grep | awk '{print $2}')
if [ -n "$pids" ]; then
    echo "$pids" | xargs kill -9 2>/dev/null
    echo "已杀死 ROS 相关进程"
else
    echo "无 ROS 进程"
fi

echo ""
echo "=== 清理占用 CAN 的 Python 进程 ==="
# 找出所有打开 can0 socket 的进程
can_pids=$(ss -f can 2>/dev/null | grep -v "Recv-Q" | awk '{print $6}' | grep -oP 'pid=\K[0-9]+' | sort -u)
if [ -z "$can_pids" ]; then
    # 备用方案: 找所有 python 进程中 import 了 piper_sdk 或 can 的
    can_pids=$(ps aux | grep python | grep -v grep | grep -v claude | grep -E "piper_sdk|python.can|C_PiperInterface" | awk '{print $2}')
fi

if [ -n "$can_pids" ]; then
    echo "发现以下进程可能占用 CAN:"
    for pid in $can_pids; do
        cmdline=$(cat /proc/$pid/cmdline 2>/dev/null | tr '\0' ' ' | head -c 100)
        echo "  PID=$pid  $cmdline"
    done
    read -p "是否全部杀死? [y/N] " confirm
    if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
        echo "$can_pids" | xargs kill -9 2>/dev/null
        echo "已杀死"
    else
        echo "跳过"
    fi
else
    echo "未发现占用 CAN 的进程"
fi

echo ""
echo "=== 检查 CAN 接口状态 ==="
ip link show can0 2>/dev/null | grep -o "state [A-Z]*"
candump_count=$(timeout 1 candump can0 2>/dev/null | wc -l)
echo "1秒内收到 ${candump_count} 帧 CAN 数据"

echo ""
echo "=== 完成 ==="
echo "现在可以安全启动: ros2 launch piper_control real_bringup.launch.py can:=can0"
