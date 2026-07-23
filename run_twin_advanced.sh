#!/bin/bash
# 数字孪生模式启动脚本（高级版本）

echo "=== 数字孪生模式（高级版本）==="
echo ""
echo "⚠️  警告：MuJoCo拖拽会控制真机！"
echo "⚠️  请确保："
echo "  1. 工作区域无障碍物"
echo "  2. 准备好物理急停按钮"
echo "  3. 首次运行建议低速测试"
echo ""

read -p "是否继续？(y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    echo "已取消"
    exit 1
fi

echo ""
echo "启动同步脚本..."
source ~/PiperSim/install/setup.bash
python3 ~/PiperSim/src/piper_mujoco/scripts/digital_twin_sync_advanced.py