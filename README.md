# PiperSim

**基于 ROS 2 的 Agilex Piper 六轴机械臂仿真与控制平台。**

PiperSim 为 Piper 机械臂提供从仿真到真机的完整工具链，目标是作为**世界模型学习**和**自主数据采集**的基础平台——在仿真中训练，采集海量操作数据，将习得的技能迁移到真实机械臂上。

---

## 功能特性

- **模块化 xacro 机器人模型**，搭配高精度 DAE 网格
- **真机控制**，基于 SocketCAN 直连（piper_sdk V2 协议）
- **Gazebo Classic 仿真**，集成 ros2_control
- **MoveIt 运动规划**（OMPL / RRTConnect）
- **手眼标定**，支持 5 种求解算法（Tsai、Park、Horaud、Andreff、Daniilidis）
- **AprilTag / ChArUco 检测**，用于标定与验证
- **Mock 硬件模式**，无需实物即可测试 MoveIt

---

## 项目结构

```
PiperSim/
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
        │   └── piper.launch.xml         # Mock / 真机启动
        └── worlds/empty_world.sdf
```

---

## 快速开始

### 环境依赖

```bash
# ROS 2 Humble
sudo apt install ros-humble-gazebo-ros2-control \
                 ros-humble-moveit \
                 ros-humble-controller-manager \
                 ros-humble-joint-trajectory-controller \
                 ros-humble-joint-state-broadcaster

# Python
pip install opencv-contrib-python numpy
```

### 编译

```bash
cd ~/PiperSim
colcon build
source install/setup.bash
```

### 启动方式

| 模式 | 命令 | 用途 |
|------|------|------|
| **Gazebo + MoveIt** | `ros2 launch piper_bringup gazebo_piper.launch.py` | 仿真与规划 |
| **Mock 硬件** | `ros2 launch piper_moveit_config demo.launch.xml` | 无硬件，测试 MoveIt |
| **真机** | `ros2 launch piper_control real_bringup.launch.py can:=can0` | 实物控制 |

---

## 真机控制

### 1. 激活 CAN 总线

```bash
bash src/piper_control/scripts/can_activate.sh can0 1000000
```

### 2. 启动控制（终端 1）

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

自动执行：清除错误 -> 使能电机 -> 设置 MOVEJ 模式 -> 回零 -> 加载控制器。

等待终端 1 出现 `=== Final status ===` 且所有控制器为 `active` 后，再开终端 2。

### 3. 发送指令（终端 2）

```bash
# 移动到指定关节角度
ros2 action send_goal /piper_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.2, 0.2, -0.2, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"

# 回零
ros2 action send_goal /piper_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
```

### 4. 停止

在终端 1 按 `Ctrl+C`，手臂会先回零再失能（安全关闭）。

---

## 手眼标定

```bash
# 终端 1：启动真机控制
ros2 launch piper_control real_bringup.launch.py can:=can0

# 终端 2：启动标定
ros2 launch piper_calibration calibration.launch.py mode:=real
```

手动移动机械臂，按空格键采集样本（建议 20 张以上），标定自动完成。

结果保存至 `data/real_eye_in_hand_result.yaml`。

### 验证标定

```bash
python3 src/piper_calibration/scripts/check_tag_position.py \
  --result-file data/real_eye_in_hand_result.yaml --tag-id 1 --tag-size 0.057
```

---

## 关节限位

| 关节 | 最小 (rad) | 最大 (rad) | 说明 |
|------|-----------|-----------|------|
| joint1 | -2.618 | 2.618 | 底座旋转 |
| joint2 | 0 | 3.14 | 肩部 |
| joint3 | -2.697 | 0 | 肘部 |
| joint4 | -1.832 | 1.832 | 腕部旋转 |
| joint5 | -1.22 | 1.22 | 腕部俯仰 |
| joint6 | -3.14 | 3.14 | 末端旋转 |
| gripper | 0 | 0.035 | 夹爪（平移） |

---

## CAN 协议（piper_sdk V2）

| 功能 | CAN ID | 说明 |
|------|--------|------|
| 电机使能/失能 | `0x471` | Byte0=电机序号(7=全部), Byte1=0x02使能 / 0x01失能 |
| 运动模式 | `0x151` | Byte0=控制模式(0x01=CAN), Byte1=运动模式(0x01=MOVEJ) |
| 关节指令 1-2 | `0x155` | 2×int32 大端序，单位 0.001° |
| 关节指令 3-4 | `0x156` | 同上 |
| 关节指令 5-6 | `0x157` | 同上 |
| 夹爪指令 | `0x159` | int32 位置(0.001mm) + uint16 力矩 + uint8 使能码 |
| 关节反馈 1-2 | `0x2A5` | 2×int32 大端序，单位 0.001°（主动推送） |
| 关节反馈 3-4 | `0x2A6` | 同上 |
| 关节反馈 5-6 | `0x2A7` | 同上 |
| 夹爪反馈 | `0x2A8` | int32 位置 + int16 力矩 + uint8 状态 |

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

---

## 开发路线

- [ ] MoveIt 真机运动规划（修复关节限位冲突）
- [ ] MuJoCo 仿真后端（高速并行训练）
- [ ] 仿真场景搭建（乒乓球桌、物体模型等）
- [ ] 仿真到真实的策略迁移流程
- [ ] 自主数据采集框架
- [ ] 世界模型集成
- [ ] 特定任务技能学习（乒乓球、抓取等）

---

## 许可证

MIT
