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

## 开始使用

### 仿真控制

> 需要 **2 个终端**。

#### 方式一：Gazebo + MoveIt

**终端 ① — 启动仿真环境：**

```bash
source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_bringup gazebo_piper.launch.py
```

等待约 10 秒，Gazebo 窗口打开，机械臂加载。

**终端 ② — 发送关节指令或启动 MoveIt：**

```bash
source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash

# 方式 A：直接发关节指令
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [{positions: [0.2, 0.2, -0.2, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"

# 方式 B：使用 MoveIt 可视化规划（与方式 A 二选一）
ros2 launch piper_moveit_config demo.launch.xml sim_gazebo:=true
```

在 RViz 中操作：
- **Planning** 标签页 → **Planning Group** 选择 `manipulator`
- 拖动末端设定目标姿态 → **Plan & Execute**
- 切换到 `gripper` 组 → 选择 `open` / `closed`pper` 组 → 选择 `open` / `closed`

#### 方式二：Mock 硬件（无 Gazebo）

无需 Gazebo 和实物，使用虚拟控制器运行 MoveIt：

```bash
source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_moveit_config demo.launch.xml
```

操作方式同上，机械臂只在 RViz 中显示。

---

### 真机控制

> **需要 2 个终端**，分别执行以下步骤。

#### 1. 激活 CAN 总线（新开终端）

```bash
# 假设 CAN 接口为 can0，波特率 1Mbps
bash src/piper_control/scripts/can_activate.sh can0 1000000
```

#### 2. 终端 ① — 启动真机控制

```bash
source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_bringup real_bringup.launch.py
```

等待终端出现以下输出（表示所有控制器已激活，可以发指令了）：

```
[spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[spawner_joint_trajectory_controller]: Configured and activated joint_trajectory_controller
[spawner_gripper_controller]: Configured and activated gripper_controller
```

> 此终端**保持运行**，不要关闭。

#### 3. 终端 ② — 发送关节指令

**新开一个终端**，执行：

```bash
source /opt/ros/humble/setup.bash
source ~/PiperSim/install/setup.bash

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

#### 4. 停止

在 **终端 ①** 按 `Ctrl+C`，手臂会先回零再失能。

---

### 手眼标定与验证

#### 1. 启动标定节点

```bash
# 终端 1：启动真机控制（如未运行）
ros2 launch piper_bringup real_bringup.launch.py

# 终端 2：启动标定节点（仿真模式）
ros2 launch piper_calibration calibration.launch.py mode:=sim

# 终端 2：启动标定节点（真机模式，需连接 RealSense 相机）
ros2 launch piper_calibration calibration.launch.py mode:=real
```

#### 2. 仿真模式（自动采集）

仿真模式下会自动生成随机位姿并采集，可以修改上面的命令，指定park方法：

```bash
ros2 launch piper_calibration calibration.launch.py mode:=sim method:=park
```

- 默认采集 15 个位姿
- 使用 `park` 方法（推荐）
- 结果保存至 `calibration_result.yaml`
- 检查数据：`calibration_result_samples.yaml`

#### 3. 真机模式（手动采集）

终端 2 运行 `mode:=real` 后，会打开实时视频窗口：

```
Calibration Capture [Space=save, Q=finish]
Captured: 0/20
SPACE=capture  Q=finish
```

操作步骤：
1. 手动移动机械臂到合适位置
2. 按 `SPACE` 采集当前位姿
3. 重复 20 次以上（建议 20 张）
4. 按 `Q` 退出采集模式，自动开始标定计算
5. 结果保存至 `data/real_eye_in_hand_result.yaml`

#### 4. 标定验证

**方式一：查看标定板位置**（仅显示，不动机械臂）

```bash
python3 src/piper_calibration/scripts/check_tag_position.py \
  --result-file data/real_eye_in_hand_result.yaml \
  --tag-id 1 \
  --tag-size 0.057
```

显示三个坐标系下的位置：camera / ee_frame / base，按 Q 退出。

**方式二：机械臂移动验证**

```bash
python3 src/piper_calibration/piper_calibration/verify_calibration_moveit.py \
  --result-file data/real_eye_in_hand_result.yaml \
  --tag-id 1 \
  --tag-size 0.057
```

操作：
1. 将 AprilTag 板子放在机械臂前方
2. 实时视频窗口显示检测到的 Tag
3. 按空格键，机械臂通过 MoveIt 移动到 Tag 上方
4. 显示实际位置与目标的误差（mm）

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
