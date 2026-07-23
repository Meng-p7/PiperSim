# PiperSim

基于 ROS 2 的 Agilex Piper 六轴机械臂仿真与控制平台。

PiperSim 为 Piper 机械臂提供从仿真到真机的完整工具链，目标是作为世界模型学习和自主数据采集的基础平台——在仿真中训练，采集海量操作数据，将习得的技能迁移到真实机械臂上。

---

## 功能特性

-  **MoveIt 集成**：完整的轨迹规划与碰撞检测
-  **多模式支持**：Mock仿真、MuJoCo物理仿真、真机控制、数字孪生
-  **MuJoCo 引擎**：高性能物理仿真，实时可视化
-  **安全机制**：位置突变检测、速度限制、紧急停止
-  **一键启动**：自动化环境配置脚本

---

## 项目结构

```
PiperSim/
├── src/
│   ├── piper_description/     # URDF 模型与可视化
│   ├── piper_moveit_config/   # MoveIt 配置
│   ├── piper_control/         # 真机控制接口
│   ├── piper_bringup/         # 启动文件
│   └── piper_mujoco/          # MuJoCo 仿真
│       ├── models/            # MuJoCo 模型（32MB）
│       ├── launch/            # 启动文件
│       ├── config/            # 控制器配置
│       └── scripts/           # 数字孪生同步脚本
├── start_sim.sh              # 仿真环境脚本
├── start_real.sh             # 真机环境脚本
└── README.md
```

---

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10 + conda
- MuJoCo 3.4.0

---

## 安装

### 1. 克隆项目
```bash
git clone https://github.com/your-username/PiperSim.git
cd PiperSim
```

### 2. 安装依赖
```bash
# 安装 ROS 2 依赖
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  python3-colcon-common-extensions

# 创建 conda 环境
conda create -n piper_sdk python=3.10 -y
conda activate piper_sdk
```

### 3. 编译项目
```bash
# 激活环境
source /opt/ros/humble/setup.bash
conda activate piper_sdk

# 清理旧编译（首次编译可跳过）
rm -rf build/ install/ log/

# 编译
colcon build --symlink-install

# Source 工作空间
source install/setup.bash
```

### 4. 安装 MuJoCo 支持（可选）

如果需要 MuJoCo 仿真：

```bash
bash install_mujoco_ros2_control_from_source.sh
```

---

## 快速开始

### 环境设置

```bash
# 仿真环境
source ~/PiperSim/start_sim.sh

# 真机环境
source ~/PiperSim/start_real.sh
```

### 基本使用

#### Mock 模式（rviz）
```bash
ros2 launch piper_moveit_config demo.launch.py mode:=mock
```

#### Sim 模式（rviz+mujoco）
```bash
ros2 launch piper_moveit_config demo.launch.py mode:=sim
```

#### Real 模式（真机控制）
```bash
# Terminal 1
bash src/piper_control/scripts/can_activate.sh can0 1000000

# Terminal 2
source ~/PiperSim/start_real.sh
ros2 launch piper_moveit_config demo.launch.py mode:=real
```

**注意**: 确保同一时间只运行一种模式。

---

## 详细文档

### Mock 模式

纯可视化模式，无物理仿真，适合快速开发。

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.xml mode:=mock
```

在 RViz 的 MotionPlanning 面板：
1. 选择 Planning Group: `manipulator`
2. 设置 Goal State
3. 点击 Plan & Execute

### Sim 模式

MuJoCo 物理仿真，支持真实物理特性。

**标准模式（MoveIt 规划）**
```bash
ros2 launch piper_moveit_config demo.launch.xml mode:=sim
```

**手动控制（GUI 拖动,目前有bug,暂不可用）**
```bash
ros2 launch piper_mujoco mujoco_manual_control.launch.py
```

### Real 模式

真机控制，通过rviz直接控制真机

**启动步骤**
```bash
# Terminal 1: CAN 激活
bash src/piper_control/scripts/can_activate.sh can0 1000000

# Terminal 2: 控制器
source ~/PiperSim/start_real.sh
ros2 launch piper_moveit_config demo.launch.py mode:=real
```

### Twin 模式（也有bug,等我修复）

数字孪生，仿真与真机同步。

**单向模式（安全）**

仅 Real → MuJoCo 单向同步

```bash
# Terminal 1: CAN激活
bash src/piper_control/scripts/can_activate.sh can0 1000000

# Terminal 2: 真机控制器
source ~/PiperSim/start_real.sh
ros2 launch piper_moveit_config demo.launch.py mode:=twin

# Terminal 3: 同步脚本
source ~/PiperSim/start_sim.sh
bash ~/PiperSim/run_twin_safe.sh
```

**双向模式**

⚠️ MuJoCo 拖拽会控制真机

```bash
# Terminal 1-2 同上
# Terminal 3
bash ~/PiperSim/run_twin_advanced.py
```

**安全机制**：
- 位置突变检测（>0.1rad）
- 斜坡限速（0.5 rad/s）
- 低通滤波平滑
- 紧急停止

---


## 技术架构

```
用户 (RViz)
    ↓ Plan & Execute
MoveIt (OMPL)
    ↓ trajectory
ros2_control
    ├─ Mock: Fake Controllers
    ├─ Sim: MuJoCo Interface
    ├─ Real: CAN Interface
    └─ Twin: Dual Interface
    ↓
机器人系统
```

## 许可证

MIT License