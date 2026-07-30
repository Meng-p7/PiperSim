# Piper MuJoCo 仿真包

`piper_mujoco` 提供 Piper 机械臂的 MuJoCo 模型、`mujoco_ros2_control`
启动文件，以及真机数字孪生所用的同步节点。项目统一入口是
`piper_moveit_config/demo.launch.py`。

本文中的命令都假定当前目录是 PiperSim 仓库根目录。

## 运行模式

| 模式 | 启动参数 | MuJoCo 依赖 | 用途 |
|---|---|---|---|
| Mock | `mode:=mock` | 不需要 | MoveIt、RViz 和假硬件联调；没有物理仿真 |
| MuJoCo 仿真 | `mode:=sim` | `mujoco_ros2_control` | MoveIt 轨迹通过 `ros2_control` 驱动 MuJoCo 模型 |
| Twin | `mode:=twin` | Python `mujoco` 和真机 CAN | 真机 `/joint_states` 经同步节点镜像到 MuJoCo GUI |

Mock 只能验证 MoveIt 和项目配置，不验证 MuJoCo 插件、模型动力学或真机接口。

Twin 不是第二套 `ros2_control` 仿真控制器。此模式使用
`piper_control/PiperHardware` 控制真机，同时由
`digital_twin_sync_realtime.py` 订阅真机 `/joint_states`，通过 MuJoCo Python
API 更新可视化模型，从而避免启动第二个 `controller_manager`。

## 安装依赖

### ROS 依赖

先加载与操作系统匹配的 ROS 2：

```bash
# Ubuntu 24.04
source /opt/ros/jazzy/setup.bash

# Ubuntu 22.04 则使用：
# source /opt/ros/humble/setup.bash
```

确认 `$ROS_DISTRO` 已设置后，可以显式安装 MuJoCo 的 ROS 集成包：

```bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
sudo apt update
sudo apt install "ros-${ROS_DISTRO}-mujoco-ros2-control"
```

`scripts/build_workspace.sh --install-deps` 也会根据当前 `$ROS_DISTRO` 使用
`rosdep` 安装包清单中的依赖。不要在同一 `build/`、`install/` 中混用 Humble
和 Jazzy。

### Twin 与独立 MuJoCo 的 Python 依赖

`mode:=sim` 使用 ROS 的 `mujoco_ros2_control`。Twin 和独立 GUI 脚本还需要
Python `mujoco`。原生环境建议使用与系统 Python 相同 ABI 的隔离环境：

```bash
sudo apt install -y python3-venv
python3 -m venv --system-site-packages .venv-mujoco
touch .venv-mujoco/COLCON_IGNORE
.venv-mujoco/bin/python -m pip install "mujoco==3.4.0"
```

不要使用 Conda Python 运行 ROS 节点。Docker 镜像已包含
`ros-jazzy-mujoco-ros2-control` 和 Python `mujoco==3.4.0`，容器内不需要重复
安装。

## 构建工作空间

从仓库根目录执行：

```bash
bash scripts/doctor.sh sim
bash scripts/build_workspace.sh --install-deps
source install/setup.bash
```

以后修改源码后可省略依赖安装：

```bash
bash scripts/build_workspace.sh
source install/setup.bash
```

如果脚本报告 `build/` 中存在另一 ROS 发行版的缓存，应先确认其中没有需要保留
的结果，再清理 `build/`、`install/`、`log/` 并重新构建。

## 启动

先让当前终端加载 ROS 和 PiperSim 工作空间：

```bash
source ./start_sim.sh
```

### Mock

```bash
ros2 launch piper_moveit_config demo.launch.py mode:=mock
```

此模式默认启动 MoveIt 和 RViz，但不加载 MuJoCo。

### MuJoCo + MoveIt

带 MuJoCo 和 RViz 窗口：

```bash
ros2 launch piper_moveit_config demo.launch.py mode:=sim
```

只关闭 MuJoCo 窗口：

```bash
ros2 launch piper_moveit_config demo.launch.py mode:=sim headless:=true
```

完全不启动 MuJoCo GUI 和 RViz：

```bash
ros2 launch piper_moveit_config demo.launch.py \
  mode:=sim rviz:=false headless:=true
```

`headless:=true` 只传给 `mode:=sim` 使用的 MuJoCo 插件；它不会自动关闭 RViz，
因此无图形会话中还应设置 `rviz:=false`。

如只需要 MuJoCo `ros2_control`，不启动 MoveIt 和 RViz，可以直接运行：

```bash
ros2 launch piper_mujoco mujoco_piper.launch.py headless:=true
```

### 独立 MuJoCo GUI

独立脚本不连接 ROS、MoveIt 或控制器，只加载仓库中的模型并打开 MuJoCo
viewer：

```bash
.venv-mujoco/bin/python src/piper_mujoco/scripts/standalone_mujoco.py
```

该脚本当前只提供 GUI 运行方式。

### Twin 数字孪生

Twin 会连接并使能真实机械臂。启动前必须清空工作区、降低速度并准备物理急停。
首次迁移到新主机时，应先完成 CAN 和真机台架验证。

```bash
source .venv-mujoco/bin/activate
source ./start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_moveit_config demo.launch.py mode:=twin can:=can0
```

Twin 当前会打开 MuJoCo viewer，尚未提供 `headless` 参数。可以用
`rviz:=false` 关闭 RViz，但这不会关闭 MuJoCo viewer：

```bash
ros2 launch piper_moveit_config demo.launch.py \
  mode:=twin can:=can0 rviz:=false
```

Twin 默认使用系统时间，因为同步节点不发布 `/clock`。

## 常用诊断

```bash
# 检查 ROS、工作空间和 MuJoCo 依赖
bash scripts/doctor.sh sim

# 确认插件可见
ros2 pkg prefix mujoco_ros2_control
ros2 pkg prefix piper_mujoco

# Sim 启动后查看控制器
ros2 control list_controllers

# Twin 启动前确认真机关节状态
ros2 topic echo /joint_states --once
```

当没有图形会话时，优先使用：

```bash
ros2 launch piper_moveit_config demo.launch.py \
  mode:=sim rviz:=false headless:=true
```

独立 MuJoCo GUI 和 Twin 当前都需要可用的图形显示环境。

## 当前验证边界

本文依据当前 launch 文件、脚本和依赖清单描述可用入口，不作“全部功能已经测试”
之类承诺。以下项目需要在目标主机上分别验收：

- Ubuntu 24.04、ROS 2 Jazzy 和 Docker 的全新环境构建；
- MuJoCo GUI 与 `headless` 启动、控制器激活及 MoveIt 轨迹执行；
- 模型碰撞、关节限制、摩擦参数和长时间实时性；
- Orbbec、NVIDIA/X11 等宿主设备和图形链路；
- 真实 CAN 适配器、Piper 固件以及 Twin 的端到端同步。

推荐按 Mock、无窗口 Sim、带 GUI Sim、最后真机 Twin 的顺序逐级验证。Mock 成功
不能作为 MuJoCo 或真机功能正常的证明。

## 目录结构

```text
piper_mujoco/
├── config/
│   └── mujoco_controllers.yaml
├── launch/
│   ├── mujoco_piper.launch.py
│   └── twin_mujoco.launch.py
├── models/
│   ├── piper.xml
│   ├── assets/
│   └── MODEL_LICENSE
└── scripts/
    ├── digital_twin_sync_realtime.py
    └── standalone_mujoco.py
```

模型及网格文件随仓库提供；模型授权信息见 `models/MODEL_LICENSE`。本 ROS 包的
许可证声明见 `package.xml`。
