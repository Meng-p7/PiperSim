#!/bin/bash
# Terminal 2: 启动同步脚本

echo "启动同步脚本..."
echo "确保Twin模式已运行（Terminal 1）"
echo ""

source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash

python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_realtime.py