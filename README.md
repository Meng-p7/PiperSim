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

- **操作系统**：Ubuntu 22.04（推荐）或 Docker
- **ROS版本**：ROS 2 Humble
- **Python**：3.10 + conda
- **MuJoCo**：3.4.0+

> **跨版本说明**：如果你使用的是 Ubuntu 24.04 或 ROS 2 Jazzy，请使用下方的 Docker 方案。

---

## 安装

### 方式一：Docker（推荐跨版本用户）

适合 Ubuntu 24.04 / ROS 2 Jazzy 用户，或希望快速体验的用户。

#### 1. 安装 Docker
```bash
# 安装 Docker
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-plugin

# 添加当前用户到docker组（避免每次sudo）
sudo usermod -aG docker $USER
# 注销后重新登录生效
```

#### 2. 允许容器访问图形界面
```bash
# 允许Docker访问X11（每次重启X server后需重新执行）
xhost +local:docker

# 或者加入 ~/.bashrc 自动执行（推荐）
echo "xhost +local:docker > /dev/null 2>&1" >> ~/.bashrc
```

#### 3. 构建并运行容器
```bash
cd PiperSim/docker
docker compose up -d

# 进入容器
docker exec -it pipersim bash
```

#### 4. 在容器内编译项目
```bash
# 进入容器后执行
cd /workspace
colcon build --symlink-install
# 注意：entrypoint.sh 会自动 source install/setup.bash，无需手动执行
```

#### 5. 安装 MuJoCo ROS2 支持（容器内）

**必选**（如果使用 Sim 或 Twin 模式）

```bash
bash install_mujoco_ros2_control_from_source.sh
```

> 如果只用 Mock 模式，可以跳过。

---

### 方式二：原生安装（Ubuntu 22.04）

适合 Ubuntu 22.04 用户，推荐作为主力开发环境。

#### 1. 安装 ROS 2 Humble

参考官方文档：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

#### 2. 安装系统依赖
```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-setup-assistant \
  python3-colcon-common-extensions \
  ethtool \
  can-utils \
  python3-pip
```

#### 3. 创建 Conda 环境并安装依赖
```bash
# 创建环境
conda create -n piper_sdk python=3.10 -y
conda activate piper_sdk

# 安装Python依赖
pip install mujoco numpy
```

**Python依赖说明**：
| 包名 | 版本 | 用途 |
|------|------|------|
| `mujoco` | ≥3.4.0 | MuJoCo物理仿真引擎 |
| `numpy` | ≥1.20 | 数值计算基础库 |

#### 4. 克隆项目
```bash
git clone https://github.com/your-username/PiperSim.git
cd PiperSim
```

#### 5. 编译项目
```bash
# 激活环境
source /opt/ros/humble/setup.bash
conda activate piper_sdk

# 编译
colcon build --symlink-install

# Source工作空间
source install/setup.bash
```

#### 6. 安装 MuJoCo ROS2 支持

**必选**（如果使用 Sim 或 Twin 模式）

```bash
bash install_mujoco_ros2_control_from_source.sh
```

> **说明**：此脚本会编译安装 `mujoco_ros2_control` 包，耗时约5-10分钟。
> 
> 如果只用 Mock 模式或 Real 模式，可以跳过此步骤。

---

### 环境变量配置（可选）

添加到 `~/.bashrc` 以简化启动：

```bash
# PiperSim快捷命令
alias piper_sim='source ~/PiperSim/start_sim.sh'
alias piper_real='source ~/PiperSim/start_real.sh'
```

---

## 快速开始

### 环境设置

启动脚本会自动完成以下操作：
- 激活 `piper_sdk` conda 环境
- Source ROS 2 Humble
- Source 编译后的工作空间
- Source mujoco_ros2_control（如果已安装）

```bash
# 仿真环境（Mock/Sim/Twin模式）
source ~/PiperSim/start_sim.sh

# 真机环境（Real模式）
source ~/PiperSim/start_real.sh
```

> **注意**：必须先完成安装步骤，启动脚本只负责环境配置，不会安装依赖。

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
ros2 launch piper_moveit_config demo.launch.py mode:=real
```

**注意**: 确保同一时间只运行一种模式。

---

## 详细文档

### Mock 模式

纯可视化模式，无物理仿真，适合快速开发。

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.py mode:=mock
```

在 RViz 的 MotionPlanning 面板：
1. 选择 Planning Group: `manipulator`
2. 设置 Goal State
3. 点击 Plan & Execute

### Sim 模式

MuJoCo 物理仿真，支持真实物理特性。

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.py mode:=sim
```

### Real 模式

真机控制，通过rviz直接控制真机

**启动步骤**
```bash
source ~/PiperSim/start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_moveit_config demo.launch.py mode:=real
```

### Twin 模式（数字孪生）

数字孪生：真机运动 → MuJoCo仿真实时跟随

**架构**：真机 `/joint_states` → 同步脚本 → MuJoCo Python API → MuJoCo GUI

**启动步骤**
```bash
source ~/PiperSim/start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_moveit_config demo.launch.py mode:=twin
```

启动后会自动打开：
- **RViz**：MoveIt规划界面
- **MuJoCo窗口**：实时跟随真机运动

**测试脚本**（可选）
```bash
# Terminal 3: 运行测试（需等待Twin模式完全启动，约15秒）
source ~/PiperSim/start_sim.sh
python3 ~/PiperSim/test_twin.py
```

---

### 手眼标定（两个月没测试过了，更改后不一定行）

相机与机械臂的手眼标定，支持仿真和真机两种模式。

#### 仿真模式（自动采样）

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_calibration calibration.launch.py mode:=sim
```

启动后：
- 自动生成随机关节姿态（使用forward_position_controller）
- 自动采集标定点
- 计算手眼变换矩阵

#### 真机模式（RealSense + 手动采样）

```bash
# Terminal 1: 激活CAN
source ~/PiperSim/start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000

# Terminal 2: 启动标定
source ~/PiperSim/start_real.sh
ros2 launch piper_calibration calibration.launch.py mode:=real
```

启动后：
1. 手动移动机械臂到不同姿态
2. 确保标定板在相机视野内
3. 按提示采集标定点
4. 自动计算手眼变换

#### 验证标定结果

```bash
# 使用MoveIt验证
python3 ~/PiperSim/src/piper_calibration/piper_calibration/verify_calibration_moveit.py
```

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