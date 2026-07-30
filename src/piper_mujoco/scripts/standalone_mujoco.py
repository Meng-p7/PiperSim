#!/usr/bin/env python3
"""
纯MuJoCo独立仿真脚本

用途：直接加载MuJoCo模型，打开GUI窗口手动控制
不依赖ROS2，不会被控制器覆盖。

启动命令：
  python3 src/piper_mujoco/scripts/standalone_mujoco.py
"""

import os
import sys
import time

try:
    import mujoco
    import mujoco.viewer
except ModuleNotFoundError:
    print("[FAIL] Python MuJoCo 模块未安装", file=sys.stderr)
    print("       修复: 先按 README 创建 .venv-mujoco，或使用 Docker", file=sys.stderr)
    sys.exit(1)


def find_model_path():
    """查找MuJoCo模型文件路径"""
    # 尝试多个可能的路径
    candidates = [
        # 源码目录
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'piper.xml'),
    ]

    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('piper_mujoco'),
            'models',
            'piper.xml',
        ))
    except Exception:
        pass
    
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
            step_started = time.monotonic()
            # 步进仿真（GUI的control滑块值会自动应用到data.ctrl）
            mujoco.mj_step(model, data)

            # 同步viewer
            viewer.sync()
            remaining = model.opt.timestep - (time.monotonic() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)

    print("\nMuJoCo viewer closed.")


if __name__ == '__main__':
    main()
