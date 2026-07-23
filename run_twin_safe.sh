#!/bin/bash
# 数字孪生模式启动脚本（实时版本）

echo "=== 数字孪生模式（实时同步）==="
echo ""
echo "功能：真机状态 → MuJoCo仿真（实时跟随）"
echo ""

echo "检查环境..."
source ~/PiperSim/install/setup.bash

echo ""
echo "启动同步脚本（30Hz实时同步）..."
python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_realtime.py