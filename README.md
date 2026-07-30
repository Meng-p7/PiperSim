# PiperSim

PiperSim 是基于 ROS 2 的 AgileX Piper 六轴机械臂开发工作区，统一提供
MoveIt 规划、Mock、MuJoCo 仿真、真机 SocketCAN 控制、数字孪生、Orbbec
相机和 Eye-to-Hand 标定。

> 当前状态（2026-07-30）：核心包已在 Ubuntu 22.04 / ROS 2 Humble
> 做过隔离构建；Ubuntu 24.04 / ROS 2 Jazzy 的 Docker 迁移配置已经整理，
> 但当前开发机没有 Docker，也没有目标真机，因此 Jazzy 容器、GPU、Orbbec、
> CAN 和实体机械臂仍须按 [ROADMAP.md](ROADMAP.md) 在目标主机验收。本文不会把
> “配置已实现”表述为“硬件已验证”。

## 功能与运行模式

| 模式 | 后端 | 是否控制真机 | 典型用途 |
|---|---|---:|---|
| `mock` | ros2_control 假硬件 | 否 | MoveIt、RViz、规划配置检查 |
| `sim` | `mujoco_ros2_control` | 否 | MuJoCo 物理仿真与轨迹执行 |
| `real` | Piper SocketCAN 硬件插件 | **是** | 真机 MoveIt 控制 |
| `twin` | 真机控制 + MuJoCo Python 镜像 | **是** | 真机运动的实时可视化 |
| 标定 | 真机只读反馈 + Orbbec | 标定采样不运动 | 固定外部相机 Eye-to-Hand |

Twin 不是只读演示模式。它会启动真机控制器，RViz 中的执行操作会作用到真实
机械臂；MuJoCo 仅镜像 `/joint_states`。

## 架构

```text
用户入口
├── start_sim.sh / start_real.sh
├── start_orbbec_camera.sh
├── scripts/doctor.sh
├── scripts/build_workspace.sh
└── docker/run.sh
        │
        ▼
MoveIt 与启动编排
├── piper_moveit_config
└── piper_bringup
        │
        ▼
ros2_control / 功能后端
├── piper_control       SocketCAN 真机接口
├── piper_mujoco       MuJoCo 与 Twin 同步
├── piper_calibration  Eye-to-Hand 标定与验证
├── piper_description  URDF/Xacro、关节与硬件描述
└── OrbbecSDK_ROS2     固定版本的相机驱动子模块
```

仓库中的主要入口：

```text
PiperSim/
├── docker/                         Jazzy 镜像、Compose 与运行入口
├── scripts/
│   ├── build_workspace.sh          rosdep + colcon
│   ├── doctor.sh                   环境/依赖诊断
│   ├── install_orbbec_udev_rules.sh
│   └── lib/common.sh               统一终端提示
├── src/
│   ├── piper_bringup/
│   ├── piper_calibration/
│   ├── piper_control/
│   ├── piper_description/
│   ├── piper_moveit_config/
│   ├── piper_mujoco/
│   └── OrbbecSDK_ROS2/
├── start_sim.sh
├── start_real.sh
├── start_orbbec_camera.sh
└── ROADMAP.md
```

## 支持与验证边界

| 环境 | 定位 | 当前结论 |
|---|---|---|
| Ubuntu 24.04 amd64 + Docker + Jazzy | 推荐迁移路径 | 配置已实现，待目标机完整验收 |
| Ubuntu 24.04 原生 Jazzy | 可选 | 脚本支持，尚未在当前环境构建 |
| Ubuntu 22.04 原生 Humble | 当前开发路径 | 核心工作区隔离构建通过 |
| Ubuntu 24.04 arm64 | 后续目标 | ROS 基础镜像支持多架构，上层依赖待验证 |
| Jetson / JetPack | 独立平台 | 不属于普通 Ubuntu arm64 的已支持范围 |

ROS 二进制包必须使用对应系统 Python。不要在运行 ROS、`rosdep` 或
`colcon build` 时激活 Conda，也不要用用户级 OpenCV/NumPy wheel 覆盖 apt
安装的版本。

## 推荐安装：Ubuntu 24.04 + Docker

以下命令面向一台干净的 Ubuntu 24.04 主机。若主机已有 Docker、容器或生产
数据，先备份 `/var/lib/docker`、Compose 配置和业务数据；不要直接执行下面
的冲突包移除步骤。

### 1. 从 Docker 官方 apt 仓库安装

不要混装 Ubuntu 的 `docker.io` 与 Docker 官方的
`docker-compose-plugin`。确认这是一台可调整容器运行时的新主机后，先移除
Docker 官方列出的冲突包，再采用
[Docker 官方 Ubuntu 安装流程](https://docs.docker.com/engine/install/ubuntu/)：

```bash
sudo apt remove $(dpkg --get-selections \
  docker.io docker-compose docker-compose-v2 docker-doc docker-buildx \
  podman-docker containerd runc | cut -f1)

sudo apt update
sudo apt install -y ca-certificates curl git
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y \
  docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo docker run --rm hello-world
sudo docker compose version
```

若第一条命令提示没有匹配的冲突包，可以继续。

本项目入口禁止以 root 运行，因此需要让当前普通用户访问 Docker daemon：

```bash
sudo usermod -aG docker "$USER"
newgrp docker  # 进入已刷新组权限的新 shell；也可以注销后重新登录
docker run --rm hello-world
docker compose version
```

这不是可忽略的项目步骤；若不希望加入高权限 `docker` 组，需要另行部署并
验收 rootless Docker（当前不在本文支持范围）。Docker 用户组等价于授予
高权限；共享主机应先阅读
[Docker post-install 安全说明](https://docs.docker.com/engine/install/linux-postinstall/)。
推荐注销并重新登录以刷新组成员身份。项目入口会拒绝
`sudo ./docker/run.sh ...`，避免容器意外以 UID 0 运行。

### 2. 克隆仓库和相机子模块

```bash
git clone --recurse-submodules https://github.com/Meng-p7/PiperSim.git
cd PiperSim
git submodule status
```

已有 clone 补齐子模块：

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

### 3. 启动 CPU 基础容器

默认配置为 CPU/headless，不要求 NVIDIA，也不共享 X11、USB 或宿主网络：

```bash
bash scripts/doctor.sh docker
./docker/run.sh config
./docker/run.sh up
./docker/run.sh shell
```

首次进入容器后：

```bash
cd /workspace
bash scripts/build_workspace.sh --install-deps
source install/setup.bash
ros2 pkg prefix piper_moveit_config
```

容器固定使用 Ubuntu 24.04 / ROS 2 Jazzy。`build/`、`install/`、`log/` 位于
Docker named volumes，不会与宿主机 Humble 产物混用；容器 UID/GID 与宿主
用户一致，也不会在源码目录留下 root 所有文件。重新进入 shell 时会自动加载
已经构建的 overlay；首次构建完成的当前 shell 仍应显式执行
`source install/setup.bash`。

无桌面环境的最小冒烟命令：

```bash
ros2 launch piper_moveit_config demo.launch.py mode:=mock rviz:=false
```

常用容器操作：

```bash
./docker/run.sh status
./docker/run.sh logs
./docker/run.sh shell
./docker/run.sh down
```

`down` 会停止容器但保留 Jazzy 构建卷。

### 4. 图形界面

仅在需要 RViz 或 MuJoCo 窗口时临时授权当前本地用户访问 X11：

```bash
xhost +si:localuser:"$(id -un)"
./docker/run.sh up --gui
```

使用结束后撤销：

```bash
xhost -si:localuser:"$(id -un)"
```

不要把宽泛的 `xhost +local:docker` 写入 `~/.bashrc`。无 X11 的主机使用
`rviz:=false headless:=true`。

### 5. 可选 NVIDIA GPU

先按
[NVIDIA Container Toolkit 官方指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
安装工具包并配置 Docker：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker run --rm --runtime=nvidia --gpus all ubuntu:24.04 nvidia-smi
```

官方样例通过后再启用 GPU override：

```bash
./docker/run.sh config --gpu
./docker/run.sh up --gpu
```

需要显示窗口时组合使用 `--gpu --gui`。没有 NVIDIA GPU 时不要传 `--gpu`。

### 6. 可选 Orbbec / CAN 真机能力

Orbbec udev 规则必须在 Ubuntu **宿主机**安装：

```bash
sudo apt install -y udev usbutils
bash scripts/install_orbbec_udev_rules.sh
# 按提示重新插拔相机
```

相机容器只映射 `/dev/bus/usb`：

```bash
./docker/run.sh up --camera
./docker/run.sh shell
```

SocketCAN 必须先在宿主机配置；容器只通过 host network 访问已经存在的 CAN
接口，不授予 `NET_ADMIN`：

```bash
sudo apt install -y can-utils ethtool iproute2
bash src/piper_control/scripts/can_activate.sh can0 1000000
./docker/run.sh up --can
./docker/run.sh shell
```

相机与 CAN 可组合为 `--camera --can`；`--hardware` 是这两个选项的兼容别名。
GPU、GUI 也可按需叠加，例如：

```bash
./docker/run.sh up --gpu --gui --camera --can
```

所有能力均为显式选择，不使用 `privileged: true`，也不挂载整个 `/dev`。
`--can` 会降低网络隔离，因此只在 SocketCAN 任务中使用。首次连接机械臂必须
先用 vcan/隔离台架验证。

## 原生安装：Humble 或 Jazzy

先按与 Ubuntu 匹配的 ROS 2 官方安装入口安装：

- Ubuntu 24.04：[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- Ubuntu 22.04：[ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)

打开一个未激活 Conda 的新终端：

```bash
# Ubuntu 24.04
source /opt/ros/jazzy/setup.bash

# Ubuntu 22.04 改为：
# source /opt/ros/humble/setup.bash

sudo apt update
sudo apt install -y \
  git python3-colcon-common-extensions python3-rosdep python3-venv \
  python3-opencv python3-numpy python3-yaml \
  can-utils ethtool iproute2 usbutils
```

首次使用 rosdep 时执行一次：

```bash
sudo rosdep init
rosdep update
```

如果 `rosdep init` 明确提示已经初始化，可以继续。随后克隆并构建：

```bash
git clone --recurse-submodules https://github.com/Meng-p7/PiperSim.git
cd PiperSim
bash scripts/build_workspace.sh --install-deps
source install/setup.bash
```

Sim 需要 ROS MuJoCo 插件：

```bash
sudo apt install "ros-${ROS_DISTRO}-mujoco-ros2-control"
```

如果已经在独立工作区从源码构建，`start_sim.sh` 会自动尝试加载
`~/mujoco_ros2_control_ws/install/setup.bash`。其他位置可显式指定：

```bash
export PIPERSIM_MUJOCO_OVERLAY=/path/to/mujoco_ros2_control_ws/install
source ./start_sim.sh
```

脚本会检查该 overlay 的 Humble/Jazzy 构建版本，不会把另一发行版的 C++ 产物
混入当前环境。

Twin 和独立 MuJoCo GUI 还需要 Python binding。原生环境使用系统 ABI 的 venv：

```bash
python3 -m venv --system-site-packages .venv-mujoco
touch .venv-mujoco/COLCON_IGNORE
.venv-mujoco/bin/python -m pip install "mujoco==3.4.0"
```

Docker 镜像已包含这两项，无需重复安装。MuJoCo 的更多说明见
[src/piper_mujoco/README.md](src/piper_mujoco/README.md)。

## 构建与诊断

统一诊断：

```bash
bash scripts/doctor.sh all
bash scripts/doctor.sh sim
bash scripts/doctor.sh real
bash scripts/doctor.sh camera
bash scripts/doctor.sh docker
```

统一构建：

```bash
# 首次安装依赖并构建
bash scripts/build_workspace.sh --install-deps

# 后续仅构建
bash scripts/build_workspace.sh

# 只构建指定包及其依赖
bash scripts/build_workspace.sh -- --packages-up-to piper_moveit_config
```

脚本会：

- 用 `[INFO]`、`[ OK ]`、`[WARN]`、`[FAIL]` 输出检查结果与修复命令；
- 根据系统和当前环境选择 Jazzy/Humble；
- 阻止 Conda 与 ROS Python ABI 混用；
- 检查另一 ROS 发行版遗留的 CMake cache；
- 用 `PYTHONNOUSERSITE=1` 避免 `~/.local` wheel 污染构建与运行。

`start_sim.sh` 与 `start_real.sh` 会改变当前 shell，所以必须使用 `source`：

```bash
source ./start_sim.sh
source ./start_real.sh
```

若误用 `bash start_sim.sh`，脚本会返回错误并给出正确命令。

## 启动

### Mock

```bash
source ./start_sim.sh
ros2 launch piper_moveit_config demo.launch.py mode:=mock
```

无 RViz：

```bash
ros2 launch piper_moveit_config demo.launch.py mode:=mock rviz:=false
```

### MuJoCo Sim

```bash
source ./start_sim.sh
ros2 launch piper_moveit_config demo.launch.py mode:=sim
```

完全无窗口：

```bash
ros2 launch piper_moveit_config demo.launch.py \
  mode:=sim rviz:=false headless:=true
```

`headless:=true` 只关闭 MuJoCo 窗口；`rviz:=false` 才会关闭 RViz。

### Real

> 以下流程会使能真实机械臂。先清空工作区、卸载负载、降低速度并确认物理急停
> 可用；新主机先完成 CAN 回环和受控台架验证。

```bash
source ./start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_moveit_config demo.launch.py mode:=real can:=can0
```

硬件插件当前默认：

- 启动时先等待完整且新鲜的 6 轴和夹爪反馈，以实测当前位置初始化；
- 以 `20%` 速度发送使能与当前位置保持命令，不再自动回零；
- 拒绝 NaN/Inf、关节限位外、单周期突变过大和反馈超时的指令；
- 停止时先发送当前位置保持，再发送失能命令，不发送回零；
- 标定模式只读反馈，不发送运动、模式切换或电机使能/失能命令。

真机、URDF、MoveIt 与 MuJoCo 共用以下保守软件限位：

| 关节 | 位置范围（rad） | 项目最大速度（rad/s） |
|---|---:|---:|
| J1 | `[-2.6179, 2.6179]` | `0.8` |
| J2 | `[0, 3.14]` | `0.8` |
| J3 | `[-2.697, 0]` | `0.8` |
| J4 | `[-1.745, 1.745]` | `0.8` |
| J5 | `[-1.22, 1.22]` | `0.8` |
| J6 | `[-2.09439, 2.09439]` | `0.8` |

它采用当前
[Piper SDK 0.6.1 参数表](https://github.com/agilexrobotics/piper_sdk/blob/081e7c588e5b79eeaefa67a0469bcc701c81014f/piper_sdk/piper_param/piper_param_manager.py#L15-L36)
与本项目原模型范围的交集；J3 因模型范围更窄而保留 `-2.697`。更换机械臂型号
或固件后必须先核对厂商规格，不能只改 MoveIt 单处放宽。

ROS 中 `gripper_joint` 的 `0–0.035 m` 表示单指行程；CAN/SDK 的
`0–0.07 m` 表示两指总开口。硬件接口在边界执行 2 倍换算，不能把这两个范围
当成冲突后直接放宽 URDF。

安全参数可通过 launch 显式调整，例如：

```bash
ros2 launch piper_moveit_config demo.launch.py \
  mode:=real can:=can0 speed_percent:=10 \
  feedback_timeout_ms:=250 max_arm_step:=0.01 max_gripper_step:=0.001
```

这些是软件防护，不是经过安全认证的控制系统，不能代替物理急停、机械限位、
受控工作区和操作规程。当前接口尚未根据机械臂状态反馈确认“使能/失能命令已
实际生效”，故障位覆盖也仍需台架核验；故障清理路径包含约 60 ms 的有界等待，
也必须纳入真机急停与失能时序验收。MoveIt SRDF 中的禁碰撞对同样必须在实体
环境逐项复核。

### Twin

```bash
source .venv-mujoco/bin/activate  # Docker 内不需要
source ./start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_moveit_config demo.launch.py mode:=twin can:=can0
```

Twin 当前始终打开 MuJoCo viewer，不支持 headless；`rviz:=false` 只关闭 RViz。
同步节点不发布 `/clock`，因此使用系统时间。

### Orbbec Femto Bolt

```bash
./start_orbbec_camera.sh
```

也可透传上游 launch 参数：

```bash
./start_orbbec_camera.sh enable_depth:=true
```

默认标定使用：

- `/camera/color/image_raw`
- `/camera/color/camera_info`

## Eye-to-Hand 标定

当前只保留真机 Eye-to-Hand：相机固定在机器人外部，AprilTag 刚性固定在末端。
建议采集 15–25 组、包含多个方向旋转的清晰样本。程序按图像时间戳查询对应
机器人 TF，并拒绝无效变换或旋转变化不足的数据。

标定采样需要三个终端：

```bash
# Terminal 1：相机
./start_orbbec_camera.sh
```

```bash
# Terminal 2：CAN + 只读标定模式
source ./start_real.sh
bash src/piper_control/scripts/can_activate.sh can0 1000000
ros2 launch piper_bringup real_bringup.launch.py \
  can:=can0 calibration_mode:=true
```

```bash
# Terminal 3：采样
source ./start_real.sh
ros2 launch piper_calibration calibration.launch.py
```

采样需要 OpenCV 图形窗口。Linux 中若 `DISPLAY` 和 `WAYLAND_DISPLAY` 都未设置，
程序会在创建窗口前明确报错并以状态码 1 退出；容器内请使用
`./docker/run.sh up --gui --hardware` 启动服务，再用
`./docker/run.sh shell` 进入容器执行标定命令。

在标定窗口中：

1. 用机械臂示教按钮进入拖动状态；
2. 让 AprilTag 清晰可见；
3. 每个不同位置和姿态按一次空格；
4. 按 Q/Esc 提前结束，至少需要 3 组有效样本；
5. 少量样本只满足算法下限，正式标定仍建议 15–25 组。

采样节点会从唯一的 transient-local `/robot_description` 发布者核对
`PiperHardware calibration_mode=true`，并持续查询
`/controller_manager/list_hardware_components` 与
`/controller_manager/list_controllers`；只有 PiperHardware 仍为 active system、
控制器类型确实为
`joint_state_broadcaster/JointStateBroadcaster`、不占用任何命令接口且没有其他
active 控制器时才会继续。采样期间若关节状态、图像或硬件/控制器安全状态失效会立即
中止。ROS 2 当前没有提供“已加载硬件参数”的不可变证明接口，因此该检查仍要求
不要在标定期间重映射或替换 `/robot_description`，也不能替代真机操作规程。

最终变换保存在 `data/real_eye_to_hand_result.yaml`，含义是：

```text
T_base_camera = base_link -> CameraInfo.header.frame_id
```

原始观测保存在同名 `_samples.yaml`。旧版
`data/real_eye_to_hand_result_samples.yaml` 没有完整末端姿态，不能恢复最终外参，
应重新采样。结果中的 child frame 来自 `CameraInfo.header.frame_id`；程序不会
自动发布可能造成 TF 多父节点的静态变换。

验证分三层，默认不会运动：

```bash
# 1. 只检查 YAML 格式与变换
ros2 run piper_calibration verify_calibration_moveit \
  --result-file data/real_eye_to_hand_result.yaml --inspect-only

# 2. 交互检测与 MoveIt 规划；按空格只规划
ros2 run piper_calibration verify_calibration_moveit \
  --result-file data/real_eye_to_hand_result.yaml

# 3. 只有清空现场并检查规划后，才显式允许真机执行
ros2 run piper_calibration verify_calibration_moveit \
  --result-file data/real_eye_to_hand_result.yaml --execute
```

执行模式仍需在检测到目标后按空格；发生 MoveIt 超时时程序会请求取消目标。

## 常见问题

### Conda 或用户级 OpenCV 污染

先确认实际加载位置：

```bash
python3 - <<'PY'
import cv2
print(cv2.__version__, cv2.__file__)
print("aruco:", hasattr(cv2, "aruco"))
print("calibrateHandEye:", hasattr(cv2, "calibrateHandEye"))
PY
```

若路径位于 Conda 或 `~/.local`，先退出 Conda，并移除冲突的用户级 OpenCV
wheel，再用 apt 恢复 `python3-opencv`。四种 OpenCV wheel 共用 `cv2`
命名空间，不要混装。项目入口会自动设置 `PYTHONNOUSERSITE=1`；若绕过入口直接
执行 ROS 命令，应先运行：

```bash
export PYTHONNOUSERSITE=1
source install/setup.bash
```

### Humble 与 Jazzy 构建产物混用

不要让两个发行版共用 `build/`、`install/`、`log/`。构建脚本检测到冲突后会
停止。确认目录中没有需要保留的产物后再清理并重建；Docker 已用 named volumes
与宿主隔离。

### Docker daemon 无权限

```bash
sudo systemctl status docker
id
docker info
```

加入 `docker` 组后需要重新登录或执行 `newgrp docker`。

### Docker 构建卷不可写

新空卷会从镜像中的空目录继承普通用户 UID/GID。若复用的旧卷根目录不可写，
entrypoint 会明确停止；若只有内部文件不可写，问题可能到构建时才暴露。先
备份卷并确认名称：

```bash
docker volume ls --filter label=com.docker.compose.project=pipersim
```

保留数据修复所有权（镜像必须已经构建）：

```bash
for volume in pipersim_pipersim_build pipersim_pipersim_install \
  pipersim_pipersim_log; do
  docker run --rm --user 0:0 --entrypoint /bin/chown \
    -v "${volume}:/volume" pipersim:jazzy \
    -R "$(id -u):$(id -g)" /volume
done
```

不要对含有需要保留数据的卷执行 `docker compose down -v`。

### 容器有 ROS、进入新 shell 却找不到工作区包

首次构建后的当前 shell 执行：

```bash
source /workspace/install/setup.bash
```

随后通过 `./docker/run.sh shell` 进入的新交互 shell 会自动加载。

### 相机找不到

```bash
git submodule status
lsusb
bash scripts/doctor.sh camera
```

确认 udev 规则安装在宿主机，重新插拔相机，并用 `--camera` 启动容器。

## 开发计划与验收

架构基线、已完成改进、P0/P1/P2 里程碑、Ubuntu 24.04 迁移测试矩阵、开发记录
模板和 Definition of Done 见 [ROADMAP.md](ROADMAP.md)。

## 许可证

六个项目自有 ROS 包的 `package.xml` 当前声明 MIT；根目录尚缺统一的
`LICENSE` 文件，正式发布前必须由版权持有人确认并补齐。MuJoCo 模型和网格的
单独授权信息见 `src/piper_mujoco/models/MODEL_LICENSE` 与
`src/piper_description/meshes/MESH_LICENSE`；Orbbec 子模块遵循其上游许可证。
