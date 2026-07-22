# PiperSim

基于 ROS 2 的 Agilex Piper 六轴机械臂仿真与控制平台。

PiperSim 为 Piper 机械臂提供从仿真到真机的完整工具链，目标是作为**世界模型学习**和**自主数据采集**的基础平台——在仿真中训练，采集海量操作数据，将习得的技能迁移到真实机械臂上。

---

## 功能特性

- 模块化 xacro 机器人模型，搭配高精度 DAE 网格
- 真机控制，基于 SocketCAN 直连（piper_sdk V2 协议）
- Gazebo Classic 仿真，集成 ros2_control
- MoveIt 运动规划（OMPL / RRTConnect）
- 手眼标定，支持 5 种求解算法（Tsai、Park、Horaud、Andreff、Daniilidis）
- AprilTag / ChArUco 检测，用于标定与验证
- Mock 硬件模式，无需实物即可测试 MoveIt

---

## 项目结构

```
PiperSim/
├── start_sim.sh              # 仿真环境一键设置脚本
├── start_real.sh             # 真机环境一键设置脚本
└── src/
    ├── piper_description/       # 机器人模型（xacro + DAE 网格）
    │   ├── urdf/                # 模块化 xacro 宏定义
    │   ├── meshes/              # 9 DAE + 12 STL + 72 OBJ 网格文件
    │   └── config/              # 控制器配置（Gazebo / Mock / 真机）
    │
    ├── piper_control/           # 真机硬件接口
    │   ├── src/piper_hardware.cpp   # C++ ros2_control 插件（SocketCAN）
    │   ├── config/piper_controllers.yaml
    │   └── scripts/             # CAN 总线激活与清理脚本
    │
    ├── piper_calibration/       # 手眼标定系统
    │   ├── piper_calibration/   # Python 模块
    │   │   ├── calibrator.py        # 5 种手眼标定算法（OpenCV）
    │   │   ├── board_detector.py    # ChArUco / AprilTag 检测
    │   │   ├── sample_collector.py  # 自动位姿采样
    │   │   └── verify_calibration.py
    │   └── launch/calibration.launch.py
    │
    ├── piper_moveit_config/     # MoveIt 运动规划配置
    │   ├── config/              # SRDF、OMPL、运动学、关节限位
    │   └── launch/              # demo.launch.xml
    │
    └── piper_bringup/           # 启动文件与仿真世界
        ├── launch/
        │   ├── gazebo_piper.launch.py   # Gazebo + MoveIt
        │   └── real_bringup.launch.py   # 真机启动
        └── worlds/empty_world.sdf
```

---

## 环境说明

本项目需要以下环境：

- **conda 环境**: `piper_sdk`（包含项目所需的所有 Python 依赖）
- **ROS 2 环境**: `fishros_humble`（ROS 2 Humble 发行版）

由于在 `~/.bashrc` 中禁用了 ROS 2 自动启动，需要手动 source。项目提供了脚本一键完成所有环境设置。

---

## 依赖安装

```bash
# ROS 2 Humble 基础依赖
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros2-control \
  ros-humble-moveit \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher

# CAN 总线相关（仅真机需要）
sudo apt install \
  can-utils \
  ethtool

# Python 依赖（标定功能）
pip install opencv-contrib-python numpy pyrealsense2 pymoveit2

# 编译工具
sudo apt install python3-colcon-common-extensions
```

> 如果已经安装依赖，可忽略。

---

## 编译

```bash
cd ~/PiperSim

# 首次编译或遇到符号链接错误时，清理后重新编译
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## 快速开始

### 一键设置环境

项目提供了两个脚本，可以一步完成所有 source 操作：

```bash
# Mock 仿真模式
source ~/PiperSim/start_sim.sh

# 真机模式
source ~/PiperSim/start_real.sh
```

脚本会自动：
1. 激活 conda 环境 `piper_sdk`
2. Source ROS 2 Humble 环境（fishros_humble）
3. Source PiperSim 工作空间

**注意**: 必须用 `source` 命令运行脚本，否则环境不会在当前 shell 生效。

---

## 使用教程

### 方式一：Mock 模式（推荐新手）

**需要 1 个终端**

无需 Gazebo 和实物，使用虚拟控制器运行 MoveIt：

**终端 1：**
```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.xml
```

等待 RViz 打开，在左侧 **MotionPlanning** 面板中：

1. **Planning Group** 选择 `manipulator`
2. **Goal State** 选择 `home` 或 `forward`
3. 点击 **Plan** 预览轨迹
4. 点击 **Plan & Execute** 执行

---

### 方式二：Gazebo 仿真（暂不可用）

> ⚠️ **状态**: Gazebo Classic 模式在当前环境中存在兼容性问题，暂不可用。
>
> 如需物理仿真，建议：
> 1. 使用 Mock 模式进行运动规划测试
> 2. 使用真机进行实际测试
> 3. 或考虑迁移到 Gazebo Ignition（新版 Gazebo）

**计划命令（暂不可用）：**

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.xml sim_gazebo:=true
```

---

### 方式三：真机控制

**需要 2 个终端**

**终端 1：激活 CAN 总线**
```bash
# 假设 CAN 接口为 can0，波特率 1Mbps
bash ~/PiperSim/src/piper_control/scripts/can_activate.sh can0 1000000
```

**终端 2：启动真机控制**
```bash
source ~/PiperSim/start_real.sh
ros2 launch piper_moveit_config demo.launch.xml real_hardware:=true
```

等待终端显示：
```
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
[spawner_gripper_controller]: Configured and activated gripper_controller
```

然后在 RViz 中操作（同 Mock 模式）。

---

### 发送关节指令（可选）

如果不想使用 MoveIt，可以直接发送关节指令：

```bash
# 移动到指定关节角度
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.2, 0.2, -0.2, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"

# 回零
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

---

## 手眼标定与验证

### 1. 启动标定节点

**需要 2 个终端**

**终端 1：启动真机控制（如未运行）**
```bash
source ~/PiperSim/start_real.sh
ros2 launch piper_bringup real_bringup.launch.py
```

**终端 2：启动标定节点**
```bash
source ~/PiperSim/start_real.sh
ros2 launch piper_calibration calibration.launch.py mode:=sim      # 仿真模式
ros2 launch piper_calibration calibration.launch.py mode:=real     # 真机模式
```

### 2. 仿真模式（自动采集）

```bash
ros2 launch piper_calibration calibration.launch.py mode:=sim method:=park
```

- 默认采集 15 个位姿
- 使用 `park` 方法（推荐）
- 结果保存至 `calibration_result.yaml`

### 3. 真机模式（手动采集）

运行后打开实时视频窗口：

```
Calibration Capture [Space=save, Q=finish]
Captured: 0/20
```

操作步骤：
1. 手动移动机械臂到合适位置
2. 按 `SPACE` 采集当前位姿
3. 重复 20 次以上
4. 按 `Q` 退出，自动计算标定结果

### 4. 标定验证

```bash
# 查看标定板位置
python3 src/piper_calibration/scripts/check_tag_position.py \
  --result-file data/real_eye_in_hand_result.yaml \
  --tag-id 1 --tag-size 0.057

# 机械臂移动验证
python3 src/piper_calibration/piper_calibration/verify_calibration_moveit.py \
  --result-file data/real_eye_in_hand_result.yaml \
  --tag-id 1 --tag-size 0.057
```

---

## 常见问题

### 残留进程导致启动失败

```bash
bash src/piper_control/scripts/clean_can.sh
```

### 电机热保护

长时间高强度运行后电机会自动锁定。停止程序，等待 3-5 分钟冷却后重新启动。

### Gazebo 插件加载失败

确认 `GAZEBO_PLUGIN_PATH` 包含 `/opt/ros/humble/lib`，终止旧 Gazebo 进程后重试。

### MoveIt 规划失败

如果显示 `Motion planning start tree could not be initialized`：

1. 确保 `base_link` 和 `table` 的碰撞已禁用（SRDF 配置）
2. 检查起始状态是否在碰撞中（RViz 中红色表示碰撞）
3. 尝试使用 **Goal State: home** 作为起始点

---

## 开发路线

- [x] MoveIt Mock 模式修复
- [x] 碰撞检测配置修复
- [ ] MoveIt 真机运动规划优化
- [ ] MuJoCo 仿真后端（高速并行训练）
- [ ] 仿真场景搭建（乒乓球桌、物体模型等）
- [ ] 仿真到真实的策略迁移流程
- [ ] 自主数据采集框架
- [ ] 世界模型集成
- [ ] 特定任务技能学习（乒乓球、抓取等）

---

## 许可证

MIT