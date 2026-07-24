#!/usr/bin/env python3
"""
纯MuJoCo独立仿真脚本

用途：直接加载MuJoCo模型，打开GUI窗口手动控制
不依赖ROS2，不会被控制器覆盖。

启动命令：
  python3 ~/PiperSim/src/piper_mujoco/scripts/standalone_mujoco.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import sys


def find_model_path():
    """查找MuJoCo模型文件路径"""
    # 尝试多个可能的路径
    candidates = [
        # 源码目录
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'piper.xml'),
        # install目录
        os.path.expanduser('~/PiperSim/install/piper_mujoco/share/piper_mujoco/models/piper.xml'),
        # 相对于脚本
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../models/piper.xml'),
    ]
    
    for path in candidates:
        abs_path = os.path.normpath(path)
        if os.path.exists(abs_path):
            return abs_path
    
    raise FileNotFoundError(
        f"MuJoCo model not found. Tried:\n" + 
        "\n".join(f"  {p}" for p in candidates)
    )


def main():
    # 查找模型
    try:
        model_path = find_model_path()
        print(f"Loading MuJoCo model: {model_path}")
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("\n" + "="*50)
    print("  MuJoCo Standalone Simulation")
    print("="*50)
    print("\nControls:")
    print("  - Left panel: Control sliders for each joint")
    print("  - Drag sliders to move the robot")
    print("  - Mouse: Rotate/zoom view")
    print("  - Close window to exit")
    print("")

    # 启动viewer（被动模式，允许GUI控制）
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置初始摄像机视角
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        while viewer.is_running():
            # 步进仿真（GUI的control滑块值会自动应用到data.ctrl）
            mujoco.mj_step(model, data)
            
            # 同步viewer
            viewer.sync()

    print("\nMuJoCo viewer closed.")


if __name__ == '__main__':
    main()