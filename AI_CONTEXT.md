# PiperSim 项目上下文

## 项目概述

ROS 2 机械臂仿真与控制平台，支持 MoveIt 覆盖、MuJoCo 物理仿真和真机控制。

## 核心组件

- `piper_description`: URDF 模型
- `piper_moveit_config`: MoveIt 配置
- `piper_control`: 真机控制
- `piper_mujoco`: MuJoCo 仿真

## 操作模式

1. **Mock**: RViz 可视化（安全）
2. **Sim**: MuJoCo 仿真
3. **Real**: 真机控制
4. **Twin**: 数字孪生

## 环境约束

- 需要 conda 环境 `piper_sdk`
- ROS 2 Humble 需手动激活
- 启动脚本必须用 `source`

## 技术栈

- ROS 2 Humble
- MoveIt 2
- MuJoCo 3.4.0
- Python 3.10