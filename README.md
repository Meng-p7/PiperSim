# PiperSim_ros2

基于 **ROS 2 Humble** 的 **Agilex Piper** 六轴机械臂工作空间，支持真机控制、Gazebo 仿真和 MoveIt 运动规划。


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
                 ros-humble-forward-command-controller \
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

**前置条件**：配置 CAN 总线

```bash
# 设置 CAN 接口（根据实际硬件调整）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 验证通信
candump can0
```

**启动控制**：

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

启动后 RViz 显示机器人实时状态，通过 `forward_position_controller` 发送位置指令：

```bash
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.57, -1.3, 0.0, 0.0, 0.0, 0.0]}"
```

**真机 + MoveIt 运动规划**：

```bash
# 终端 1：启动真机控制
ros2 launch piper_control real_bringup.launch.py

# 终端 2：启动 MoveIt
ros2 launch piper_moveit_config move_group.launch.py
```

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

基于 ros2_control 的 C++ 硬件接口插件，通过 SocketCAN 与实物 Piper 机械臂通信。

通信流程：

```
controller_manager 加载 PiperHardware 插件
  → on_activate: 发送使能命令（CAN ID 0x010）
  → 每秒 50 次循环：
      read():  发送轮询 → 收 4 帧 CAN 数据 → 存入 hw_pos_[]
      write(): 读 hw_cmd_[] → 编码为 3+1 帧 CAN 数据发出
```

CAN 协议：
- 机械臂关节：3 帧（关节 1-2、3-4、5-6），大端 int32，缩放因子 57295.7795
- 夹爪：1 帧，位置单位微米，速度 1000
- 使能命令：ID 0x010，byte0 = 0x01
- 状态响应：ID 0x201-0x203（关节）、0x205（夹爪）

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
