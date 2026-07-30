# PiperSim 后续开发规划

目标交付平台：**Ubuntu 24.04 LTS / amd64 / ROS 2 Jazzy**。Docker 作为首选
部署方式，原生安装作为调试与硬件排障的补充路径。

## P0：完成目标主机迁移

目标：在一台全新的 Ubuntu 24.04 amd64 主机上，仅按 README 即可完成安装、
构建和无硬件运行。

- [ ] 从空目录 clone 仓库并递归初始化子模块。
- [ ] 按 Docker 官方 apt 仓库方式安装 Engine、Buildx 和 Compose plugin。
- [ ] 验证普通用户运行 Docker，不依赖 root 启动项目。
- [ ] 构建 Jazzy 镜像，并在容器内完成 `rosdep`、全量 `colcon build` 和测试。
- [ ] 验证 CPU/headless 默认路径不要求 GPU、DISPLAY、USB 或 CAN。
- [ ] 验证 Mock headless；在桌面主机上补充 RViz 和 MuJoCo GUI 验收。
- [ ] 检查容器生成文件的 UID/GID，确保宿主工作区不出现 root 所有文件。
- [ ] 在干净主机上逐条复现 README，并修正遗漏的依赖、命令和清理步骤。
- [ ] 固定发布所用镜像、ROS 包、Python 依赖和 Orbbec 子模块版本。
- [ ] 由版权持有人确认许可范围，并补充根目录 `LICENSE`。

## P1：完成外设与真机验收

目标：在 amd64 工作站上完成 GPU、Orbbec、SocketCAN 和 Piper 真机的受控
验收。

- [ ] 验证 NVIDIA Container Toolkit、GPU Compose、MuJoCo 和 RViz。
- [ ] 验证 Orbbec Femto Bolt 的 udev、USB 热插拔、图像及 `camera_info`。
- [ ] 先用独立 vcan 接口验证宿主与容器间的 SocketCAN 收发。
- [ ] 使用真实 CAN 适配器验证 bitrate、断线、重连和错误提示。
- [ ] 按“只读反馈 → 单关节低速 → MoveIt 规划 → 低速执行”的顺序验收 Real。
- [ ] 验证 Twin 的真机状态与 MuJoCo 同步。
- [ ] 验证 Calibration 的采样、求解、规划预览和显式执行安全门。
- [ ] 验证 Ctrl+C、节点异常、CAN 掉线、相机掉线和物理急停后的安全状态。
- [ ] 根据实测结果缩小容器 sudo、X11 和 host network 权限。
- [ ] 建立 amd64 CI：ShellCheck、Compose 配置检查、构建、测试和 Mock/Sim 冒烟。

## P2：发布与长期维护

- [ ] 增加 Ubuntu 24.04 arm64 CPU 镜像及 Mock/Sim headless 验收。
- [ ] 为 Jetson/JetPack 建立独立镜像、驱动矩阵和 GPU 测试流程。
- [ ] 建立 Orbbec、CAN 和机械臂硬件在环回归测试台。
- [ ] 增加故障注入测试：反馈超时、CAN 断链、相机掉线和节点重启。
- [ ] 评估 rootless Docker、DDS discovery 和取消 host network 的可行性。
- [ ] 优化镜像体积与构建缓存，并加入 SBOM、依赖审计和安全扫描。
- [ ] 建立数据集目录、元数据、版本和校验规则，为后续模型训练提供稳定接口。
- [ ] 发布稳定版，并提供升级、兼容性和回滚说明。

## 验收矩阵

所有项目都必须在对应目标平台实际运行并保存版本、命令和日志；仅通过编译不算
完成。

| ID | 平台与能力 | 验收内容 | 优先级 |
|---|---|---|---|
| T01 | amd64 / CPU / headless | Docker 安装、镜像构建、全量编译、Mock headless | P0 |
| T02 | amd64 / CPU / X11 | RViz Mock、MuJoCo CPU GUI、X11 权限回收 | P0 |
| T03 | amd64 / NVIDIA / X11 | GPU 容器、`nvidia-smi`、MuJoCo、RViz | P1 |
| T04 | amd64 / Orbbec | udev、热插拔、彩色/深度图像、相机内参 | P1 |
| T05 | amd64 / vcan 与真实 CAN | 双向收发、断线重连、错误恢复 | P1 |
| T06 | amd64 / Orbbec / Piper CAN | Real、Twin、Calibration 端到端与急停 | P1 |
| T07 | arm64 / CPU / headless | 镜像、全量编译、Mock/Sim headless | P2 |
| T08 | Jetson / NVIDIA | JetPack/L4T 专用镜像与 GPU 路径 | P2 |

真机 T06 必须空载、低速、有限工作区运行，并由操作者全程掌握物理急停；不得
用自动脚本跳过前置验收阶段。

## Definition of Done

一个里程碑只有在以下条件全部满足后才算完成：

- [ ] 对应验收矩阵项目全部通过，并保留可追溯的环境版本、命令和日志。
- [ ] 从干净环境可重复安装、构建、启动和清理。
- [ ] 脚本失败时返回非零状态，并给出明确原因和可执行的修复建议。
- [ ] 新依赖已写入 Dockerfile、包清单或 README，不依赖开发机隐式环境。
- [ ] 未混用 Humble/Jazzy、Conda/系统 Python 或宿主/容器构建产物。
- [ ] GPU、USB、CAN、X11 和宿主网络权限均按需显式启用。
- [ ] 真机异常退出、断链和急停后的状态已经实机确认。
- [ ] README、脚本帮助和发布支持范围与实际行为一致。
