# PiperSim_ros2

基于 **ROS 2 Humble** 的 **Agilex Piper** 六轴机械臂工作空间，支持真机控制、Gazebo 仿真和 MoveIt 运动规划。真机通信协议与 **piper_sdk V2**（官方 SDK）完全一致。


## 工作空间结构

```
PiperSim_ros2/
  src/
    piper_description/       # 机器人模型、URDF、网格文件、RViz 配置
    piper_control/           # ros2_control CAN 总线硬件接口（C++ 插件）
    piper_calibration/       # 手眼标定（真机手动采集 / 仿真自动采集）
    piper_moveit_config/     # MoveIt 运动规划配置
```

## 三种运行模式

| 模式 | URDF | 硬件插件 | 用途 |
|------|------|----------|------|
| 真机 | `piper.urdf` | `piper_control/PiperHardware` (SocketCAN) | 连接实物机械臂 |
| Gazebo 仿真 | `piper_gazebo.urdf` | `gazebo_ros2_control/GazeboSystem` | 仿真环境 + MoveIt 规划 |
| Fake 硬件 | `piper_fake.urdf` | `fake_components/GenericSystem` | 无硬件测试 MoveIt |

---

## 环境要求

- **ROS 2 Humble**（Ubuntu 22.04）
- **colcon** 构建工具
- **Gazebo Classic 11**（仿真模式）
- **SocketCAN** 支持（真机模式）
- **OpenCV**（含 ArUco 模块，标定使用）
- **pyrealsense2**（真机标定，RealSense 相机驱动）

安装依赖：

```bash
# ROS2 包依赖
sudo apt install ros-humble-gazebo-ros2-control \
                 ros-humble-moveit \
                 ros-humble-controller-manager \
                 ros-humble-joint-trajectory-controller \
                 ros-humble-joint-state-broadcaster

# Python 依赖
pip install opencv-contrib-python numpy pyrealsense2
```

## 构建

```bash
cd ~/PiperSim_ros2
colcon build
source install/setup.bash
```

---

## 快速开始

### 1. 只看模型（不需要机器人和仿真）

最安全的验证方式，拖动滑块即可转动各关节：

```bash
ros2 launch piper_description display.launch.py
```

### 2. Fake 硬件 + MoveIt（推荐先试这个）

无需真实硬件和 Gazebo，使用 fake 关节测试 MoveIt 运动规划：

```bash
ros2 launch piper_moveit_config moveit_demo.launch.py
```

在 RViz 中拖动机械臂末端，点击 **Plan & Execute** 测试运动规划。

### 3. Gazebo 仿真 + MoveIt

启动 Gazebo 仿真环境，支持 MoveIt 运动规划：

```bash
ros2 launch piper_moveit_config gazebo_demo.launch.py
```

可选：启动 Gazebo 图形界面（需要 GPU）：

```bash
ros2 launch piper_moveit_config gazebo_demo.launch.py gui:=true
```

### 4. 控制真机

**前置条件**：配置 CAN 总线（波特率 1000000）

```bash
# 方法一：使用项目自带脚本（推荐，自动检测 USB CAN 设备并配置）
bash src/piper_control/scripts/can_activate.sh can0 1000000

# 方法二：手动配置
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证通信
candump can0
```

**启动控制**：

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

启动后自动完成：CAN 使能 → 设置 MOVEJ 模式 → 加载 `piper_arm_controller` + `piper_gripper_controller` + `joint_state_broadcaster` → RViz 显示实时状态。

**方式 A：直接用 ROS2 命令行控制关节**

```bash
# 发送关节目标（使用 JointTrajectoryController 的 action 接口）
ros2 action send_goal /piper_arm_controller/follow_joint_trajectory \
  trajectory_msgs/JointTrajectory \
  "{joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.0, 1.57, -1.3, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

**方式 B：真机 + MoveIt 运动规划**（推荐）

```bash
# 终端 1：启动真机控制
ros2 launch piper_control real_bringup.launch.py can:=can0

# 终端 2：启动 MoveIt
ros2 launch piper_moveit_config move_group.launch.py
```

在 RViz 中拖动机械臂末端，点击 **Plan & Execute** 即可控制真机运动。

### 真机 CAN 协议说明（piper_sdk V2）

本项目真机通信协议与 Agilex 官方 `piper_sdk` V2 完全一致：

| 功能 | CAN ID | 说明 |
|------|--------|------|
| 电机使能/失能 | `0x471` | Byte0=电机序号(7=全部), Byte1=0x02使能/0x01失能 |
| 运动模式控制 | `0x151` | Byte0=控制模式(0x01=CAN), Byte1=运动模式(0x01=MOVEJ), Byte2=速度百分比 |
| 关节命令 1-2 | `0x155` | 2×int32 big-endian, 单位 0.001° |
| 关节命令 3-4 | `0x156` | 同上 |
| 关节命令 5-6 | `0x157` | 同上 |
| 夹爪命令 | `0x159` | int32 位置(0.001mm) + uint16 力矩(0.001N/m) + uint8 使能码 |
| 关节反馈 1-2 | `0x2A5` | 2×int32 big-endian, 单位 0.001° (臂主动推送, ~200Hz) |
| 关节反馈 3-4 | `0x2A6` | 同上 |
| 关节反馈 5-6 | `0x2A7` | 同上 |
| 夹爪反馈 | `0x2A8` | int32 位置(0.001mm) + int16 力矩 + uint8 状态 |

---

## 手眼标定

### 仿真标定（自动采集）

```bash
# 先启动 Gazebo 仿真
ros2 launch piper_moveit_config gazebo_demo.launch.py

# 终端 2：启动标定节点
ros2 launch piper_calibration calibration.launch.py mode:=sim
```

流程：程序生成 15 个随机位姿 → MoveIt 逐个移动 → 每到位自动拍照检测 ChArUco 标定板 → 采集完成后自动计算标定结果。

### 真机标定（手动采集）

**步骤一**：启动真机控制（终端 1）

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

**步骤二**：启动标定节点（终端 2）

```bash
ros2 launch piper_calibration calibration.launch.py mode:=real
```

**步骤三**：数据采集

1. 弹出 OpenCV 窗口，实时显示 RealSense 相机画面
2. 手动移动机械臂到不同位姿（确保标定板在画面中清晰可见）
3. 按**空格键**采集（保存图片 + 记录当前关节角度）
4. 画面左上角显示进度，如 `Captured: 5/20`
5. 重复移动 → 采集，直到采满 20 张
6. 按 **Q** 或 **Esc** 可提前结束（至少需要 3 张）

**步骤四**：自动标定

采集结束后程序自动：
- 检测每张图片中的 ChArUco 标定板
- 计算手眼标定矩阵
- 结果保存为 YAML 文件，同时发布静态 TF 变换

标定结果输出：
- `real_eye_in_hand_result.yaml` — 4x4 变换矩阵（相机到末端执行器）
- 静态 TF：`link6` → `wrist_cam_link`

### 标定参数说明

| 参数 | 仿真 | 真机 |
|------|------|------|
| 相机来源 | Gazebo 模拟相机 | RealSense（pyrealsense2 直连） |
| 采集方式 | 自动生成随机位姿 + MoveIt 移动 | 手动拖动机械臂 + 空格键采集 |
| 标定板 | ChArUco 9x14，方格 30mm | ChArUco 9x14，方格 20mm |
| 采集数量 | 15 | 20 |
| 标定方法 | Park | Park |

支持的标定方法：`tsai`、`park`、`horaud`、`andreff`、`daniilidis`（在配置文件中修改 `method` 参数）。

---

## 功能包详解

### piper_description

机器人描述包。提供 URDF 模型、3D 网格文件（STL）、RViz 配置。

机器人模型：
- 6 个旋转关节（joint1-joint6），组成机械臂
- 1 个平移夹爪，平行夹持结构（joint7 控制左指，joint8 镜像跟随）
- 1 个固定腕部相机坐标系（`wrist_cam_link`）

URDF 文件：
- `piper.urdf` — 真机使用，包含 `PiperReal` 硬件声明
- `piper_gazebo.urdf` — Gazebo 仿真使用，碰撞用简单几何体
- `piper_fake.urdf` — Fake 硬件测试使用

### piper_control

基于 ros2_control 的 C++ 硬件接口插件，通过 SocketCAN 与实物 Piper 机械臂通信。协议与 piper_sdk V2 完全一致。

通信流程：

```
controller_manager 加载 PiperHardware 插件
  → on_configure: 打开 CAN socket
  → on_activate:
      1. 发送使能命令 (ID 0x471, motor_num=7, enable=0x02)
      2. 发送运动模式 (ID 0x151, ctrl=CAN, mode=MOVEJ, speed=100%)
      3. 读取初始关节位置作为命令起点
  → 每秒 50 次循环：
      read():  drain_can_rx() 非阻塞读取所有待处理 CAN 帧
               解析 0x2A5/0x2A6/0x2A7(关节) 和 0x2A8(夹爪) 反馈
      write(): 发送 0x155/0x156/0x157(关节) 和 0x159(夹爪) 命令
  → on_deactivate: 发送失能命令 (ID 0x471, enable=0x01)
```

### piper_moveit_config

MoveIt 运动规划配置包。

Launch 文件：
- `moveit_demo.launch.py` — Fake 硬件 + MoveIt（无需真机和 Gazebo）
- `gazebo_demo.launch.py` — Gazebo 仿真 + MoveIt
- `move_group.launch.py` — MoveIt 规划节点（配合真机使用）

配置文件：
- `piper.srdf` — 语义机器人描述（运动组、末端执行器）
- `kinematics.yaml` — KDL 运动学求解器
- `ompl_planning.yaml` — OMPL 规划器配置
- `joint_limits.yaml` — 关节限位

### piper_calibration

手眼标定系统，基于 ChArUco 标定板和 OpenCV 的 `calibrateHandEye` 算法。

核心模块：
- `calibration_node.py` — 主节点，调度整个标定流程
- `calibrator.py` — 标定算法封装（5 种方法、SVD 修正、误差计算）
- `board_detector.py` — 标定板检测（ChArUco / 棋盘格）
- `sample_collector.py` — 仿真模式的随机位姿生成

配置文件（`src/piper_calibration/config/`）：

| 文件 | 说明 |
|------|------|
| `calibration_params.yaml` | 仿真标定参数 |
| `real_calibration_params.yaml` | 真机标定参数（RealSense） |

---

## 常见问题

### Gazebo 仿真插件加载失败

如果看到 `Could not contact service /controller_manager/list_controllers`，说明 gazebo_ros2_control 插件没有加载。检查：
- `GAZEBO_PLUGIN_PATH` 是否包含 `/opt/ros/humble/lib`
- 终止旧的 Gazebo 进程后重试

### 真机 CAN 通信失败

```bash
# 检查 CAN 接口状态
ip link show can0

# 检查是否有数据
candump can0

# 如果没有数据，检查接线和波特率
sudo ip link set can0 type can bitrate 1000000
```

### 标定板检测失败

- 确保标定板参数与实际标定板一致（`charuco_rows`、`charuco_cols`、`charuco_square_length`）
- 确保光线充足，标定板无反光
- 标定板需要完整出现在画面中
