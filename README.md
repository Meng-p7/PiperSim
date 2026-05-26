# PiperSim_ros2

基于 **ROS 2** 的 **Agilex Piper** 六轴机械臂工作空间，提供 URDF 机器人描述、CAN 总线硬件接口和自动手眼标定系统。支持实物控制和 Gazebo 仿真两种模式。

## 工作空间结构

```
PiperSim_ros2/
  src/
    piper_description/     # 机器人模型、URDF、网格文件、可视化
    piper_control/         # ros2_control CAN 总线硬件接口（C++ 插件）
    piper_calibration/     # 手眼标定（实物手动采集 / 仿真自动采集）
```

## 环境要求

- **ROS 2**（Humble / Iron / Jazzy）
- **colcon** 构建工具
- **SocketCAN** 支持（Linux 内核，实物控制需要）
- **OpenCV**（含 ArUco 模块，标定使用）
- **pyrealsense2**（实物标定使用，RealSense 相机驱动）
- **pymoveit2**（仿真标定的运动规划使用）

安装 Python 依赖：

```bash
pip install opencv-contrib-python numpy pyrealsense2 pymoveit2
```

## 构建

```bash
cd PiperSim_ros2
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

### 2. Gazebo 仿真

启动仿真环境，机器人加载到 Gazebo 中：

```bash
ros2 launch piper_calibration sim_setup.launch.py
```

另开终端发送运动指令：

```bash
source install/setup.bash
ros2 topic pub --once /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 1.57, -1.3, 0.0, 0.0, 0.0, 0.0]}"
```

### 3. 控制实物机器人

前置条件：配置 CAN 总线

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

启动控制：

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

启动后 RViz 显示机器人实时状态，同样可以通过 `ros2 topic pub` 发送位置指令。

---

## 手眼标定

### 仿真标定（自动采集）

仿真模式下由程序自动生成随机位姿，通过 MoveIt 移动机器人，自动检测标定板并采集数据。

```bash
# 启动 Gazebo 仿真 + 标定节点
ros2 launch piper_calibration calibration.launch.py mode:=sim
```

流程：程序生成 15 个随机位姿 → MoveIt 逐个移动 → 每到位自动拍照检测 ChArUco 标定板 → 采集完成后自动计算标定结果。

### 实物标定（手动采集）

实物模式下用户手动拖动机械臂（松灵 Piper 支持按钮切换自由移动模式），通过 OpenCV 窗口实时查看相机画面，按空格键采集图片和关节角度。

**步骤一**：启动实物控制（终端 1）

```bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

**步骤二**：启动标定节点（终端 2）

```bash
ros2 launch piper_calibration calibration.launch.py mode:=real
```

**步骤三**：操作标定

1. 弹出 OpenCV 窗口，实时显示 RealSense 相机画面
2. 点击机械臂上的按钮，进入自由移动模式
3. 将标定板放到相机视野中
4. 再次点击按钮固定机械臂
5. 按**空格键**采集（保存原始图片 + 记录当前关节角度）
6. 画面左上角显示进度，如 `Captured: 5/20`
7. 重复移动 → 固定 → 空格采集，直到采满 20 张
8. 按 **Q** 或 **Esc** 可提前结束（至少需要 3 张）
9. 采集结束后程序自动检测标定板并计算标定结果
10. 结果保存为 YAML 文件，同时发布静态 TF 变换

标定结果输出：
- `real_eye_in_hand_result.yaml` — 4x4 变换矩阵（相机到末端执行器）
- 静态 TF：`link6` → `wrist_cam_link`

### 标定参数说明

| 参数 | 仿真 | 实物 |
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

机器人描述包。提供 URDF 模型、3D 网格文件（STL/OBJ）、RViz 配置。

机器人模型：
- 6 个旋转关节（joint1-joint6），组成机械臂
- 1 个平移夹爪，平行夹持结构（joint7 控制左指，joint8 镜像跟随）
- 1 个固定腕部相机坐标系（`wrist_cam_link`）

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
| `real_calibration_params.yaml` | 实物标定参数（RealSense） |
| `joint_limits.yaml` | 各关节限位 |
| `kinematics.yaml` | KDL 运动学求解器配置 |
| `piper.srdf` | 语义机器人描述（运动组、末端执行器） |

## 主要依赖

| 依赖 | 用途 |
|------|------|
| `hardware_interface` / `pluginlib` | ros2_control 插件框架 |
| `controller_manager` / `forward_command_controller` | 控制器管理 |
| `robot_state_publisher` | 根据 URDF 发布 TF |
| `cv_bridge` / `sensor_msgs` | 相机图像处理 |
| `pyrealsense2` | RealSense 相机直连（实物标定） |
| `pymoveit2` | Python MoveIt2 封装（仿真标定） |
| `OpenCV`（ArUco） | 标定板检测与手眼标定 |
| `SocketCAN` | 与实物硬件的 CAN 总线通信 |
