# PiperSim_ros2

基于 **ROS 2 Humble** 的 **Agilex Piper** 六轴机械臂工作空间，支持真机控制、Gazebo 仿真和手眼标定。

---

## 项目架构

```
PiperSim_ros2/
├── src/
│   ├── piper_description/       # 机器人模型
│   │   ├── urdf/                # 3 个 URDF（真机/仿真/Fake）
│   │   ├── meshes/              # STL/OBJ 网格文件
│   │   ├── launch/              # display.launch.py（仅看模型）
│   │   └── rviz/                # RViz 配置
│   │
│   ├── piper_control/           # 真机硬件接口
│   │   ├── src/piper_hardware.cpp   # C++ ros2_control 插件（SocketCAN）
│   │   ├── config/piper_controllers.yaml
│   │   ├── launch/real_bringup.launch.py
│   │   └── scripts/             # can_activate.sh, clean_can.sh
│   │
│   ├── piper_calibration/       # 手眼标定
│   │   ├── piper_calibration/   # Python 模块
│   │   │   ├── calibration_node.py    # 标定主节点
│   │   │   ├── calibrator.py          # 标定算法（5种方法）
│   │   │   ├── board_detector.py      # ChArUco 检测
│   │   │   ├── sample_collector.py    # 仿真随机位姿生成
│   │   │   └── verify_calibration.py  # 标定验证（MuJoCo IK）
│   │   ├── launch/calibration.launch.py
│   │   ├── config/              # 标定参数
│   │   └── scripts/check_tag_position.py  # Tag 位置查看工具
│   │
│   └── piper_moveit_config/     # MoveIt 运动规划配置
│       ├── config/              # SRDF、OMPL、运动学、关节限位
│       └── launch/              # moveit_demo / gazebo_demo / move_group
│
└── data/                        # 标定数据（标定结果 + 采集图片）
```

## 三种运行模式

| 模式 | URDF | 硬件插件 | 用途 |
|------|------|----------|------|
| 真机 | `piper.urdf` | `piper_control/PiperHardware` (SocketCAN) | 连接实物 |
| Gazebo 仿真 | `piper_gazebo.urdf` | `gazebo_ros2_control/GazeboSystem` | 仿真 + MoveIt |
| Fake 硬件 | `piper_fake.urdf` | `fake_components/GenericSystem` | 无硬件测试 MoveIt |

---

## 环境要求

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

## 真机控制

### 1. 激活 CAN 总线

```bash
bash src/piper_control/scripts/can_activate.sh can0 1000000
```

### 2. 启动真机控制（终端 1）

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

启动后自动：清除错误 → 使能电机 → 设置 MOVEJ 模式 → 回零 → 加载控制器

等终端 1 出现 `=== Final status ===` 且三个控制器都是 `active` 后，再开下一个终端。

### 3. 控制机械臂（终端 2）

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

终端 1 按 `Ctrl+C`，手臂会先回零再失能（安全关闭）。

---

## 手眼标定

### 标定（终端 1 + 终端 2）

**终端 1**：启动真机控制
```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

**终端 2**：启动标定采集
```bash
ros2 launch piper_calibration calibration.launch.py mode:=real
```

操作：弹出相机画面 → 手动移臂 → 空格键拍照 → 采满 20 张自动计算

结果保存到 `data/real_eye_in_hand_result.yaml`

### 验证标定（终端 1 + 终端 2）

**终端 1**：同上（real_bringup）

**终端 2**：
```bash
python3 src/piper_calibration/scripts/check_tag_position.py \
  --result-file data/real_eye_in_hand_result.yaml --tag-id 1 --tag-size 0.057
```

把标定板放在桌面上，看 `base` 的 Z 值是否接近桌面高度。

### 验证标定精度（终端 1 + 终端 2）

**终端 1**：同上（real_bringup）

**终端 2**：
```bash
python3 src/piper_calibration/piper_calibration/verify_calibration.py \
  --result-file data/real_eye_in_hand_result.yaml --tag-id 1 --tag-size 0.057
```

按空格键，机械臂移到标定板上方 10cm，终端显示误差。

---

## 仿真模式

### Fake 硬件 + MoveIt（推荐先试）

```bash
ros2 launch piper_moveit_config moveit_demo.launch.py
```

在 RViz 中拖动末端，点击 Plan & Execute 测试运动规划。

### Gazebo 仿真 + MoveIt

```bash
ros2 launch piper_moveit_config gazebo_demo.launch.py

# 带图形界面（需要 GPU）
ros2 launch piper_moveit_config gazebo_demo.launch.py gui:=true
```

### 仅看模型

```bash
ros2 launch piper_description display.launch.py
```

---

## 关节限位

| 关节 | 最小 (rad) | 最大 (rad) | 功能 |
|------|-----------|-----------|------|
| joint1 | -2.618 | 2.618 | 底座旋转 |
| joint2 | 0 | 3.14 | 肩部 |
| joint3 | -2.697 | 0 | 肘部 |
| joint4 | -1.832 | 1.832 | 腕部旋转 |
| joint5 | -1.22 | 1.22 | 腕部俯仰 |
| joint6 | -3.14 | 3.14 | 末端旋转 |
| joint7 | 0 | 0.035 | 夹爪（平移） |
| joint8 | -0.035 | 0 | 夹爪镜像（mimic） |

---

## 常见问题

### 残留进程导致启动失败

```bash
bash src/piper_control/scripts/clean_can.sh
```

### 电机不动 / CAN 命令无响应

Piper 有**电机热保护**，连续高强度运行后电机会自动锁定。

症状：控制器报 `SUCCEEDED` 但关节角度不变。

解决：停止程序，等 3-5 分钟冷却，重新启动。

### Gazebo 仿真插件加载失败

检查 `GAZEBO_PLUGIN_PATH` 是否包含 `/opt/ros/humble/lib`，终止旧 Gazebo 进程后重试。

---

## CAN 协议（piper_sdk V2）

| 功能 | CAN ID | 说明 |
|------|--------|------|
| 电机使能/失能 | `0x471` | Byte0=电机序号(7=全部), Byte1=0x02使能/0x01失能 |
| 运动模式控制 | `0x151` | Byte0=控制模式(0x01=CAN), Byte1=运动模式(0x01=MOVEJ) |
| 关节命令 1-2 | `0x155` | 2×int32 big-endian, 单位 0.001° |
| 关节命令 3-4 | `0x156` | 同上 |
| 关节命令 5-6 | `0x157` | 同上 |
| 夹爪命令 | `0x159` | int32 位置(0.001mm) + uint16 力矩 + uint8 使能码 |
| 关节反馈 1-2 | `0x2A5` | 2×int32 big-endian, 单位 0.001° (臂主动推送) |
| 关节反馈 3-4 | `0x2A6` | 同上 |
| 关节反馈 5-6 | `0x2A7` | 同上 |
| 夹爪反馈 | `0x2A8` | int32 位置 + int16 力矩 + uint8 状态 |

---

## 待办事项

### MoveIt 真机运动规划

`move_group.launch.py` 配合真机使用时，规划器报 `STATUS_ABORTED` / `Start state violates joint limits`。

待排查：
- CHOMP 规划器对起始状态要求严格，需切换到 OMPL (RRTConnect)
- `move_group` 和 `real_bringup` 的控制器配置需统一
- 关节限位在 URDF 和 MoveIt 配置中需保持一致

### Gazebo 仿真完善

- 验证 `gazebo_demo.launch.py` 在当前环境下能否正常启动
- 确认 Gazebo 控制器配置与 MoveIt 兼容
- 测试仿真模式下的 MoveIt 运动规划

### MuJoCo + ROS2 仿真（备选方案）

如果 Gazebo 有问题，可用 MuJoCo 作为仿真后端：

- 参考项目 `~/桌面/PiperSim/` 已有完整的 MuJoCo 模型和控制器
- 需要安装 `mujoco` 和 `ros2_control` 的 MuJoCo 插件（如 `mujoco_ros2_control`）
- 参考：https://github.com/mujoco-mvp/mujoco_ros2_control
