# PiperSim项目速览

> 给 AI 快速建立全局认知的文档。真值以代码为准,本文只给"不读全代码就看不出"的关键信息与索引。

## 一句话
基于 ROS 2 Humble 的 Agilex Piper 六轴机械臂 + 平行夹爪 仿真/控制平台。目标:仿真训练 → 数据采集 → 迁移真机(世界模型 / 自主采集)。

## 技术栈
ROS 2 Humble · Gazebo Classic(`gazebo_ros2_control`)· MoveIt2(OMPL/RRTConnect)· 真机 SocketCAN(piper_sdk V2 协议)· 手眼标定 OpenCV。

## 包结构(`src/`)
- **piper_description** — 机器人模型:模块化 xacro + 网格(stl/dae/obj)+ 控制器 yaml(`gz_/mock_/` + real 在 piper_control)
- **piper_control** — 真机硬件接口:C++ ros2_control 插件(`src/piper_hardware.cpp`,SocketCAN)+ CAN 激活/清理脚本
- **piper_calibration** — 手眼标定(5 算法)+ ChArUco/AprilTag 检测(Python)
- **piper_moveit_config** — MoveIt 配置(SRDF/OMPL/运动学/限位/`demo.launch.xml`)
- **piper_bringup** — 启动文件 + 仿真世界

## 启动脚本（重要）
项目根目录提供两个脚本，一键完成所有 source 操作：

```bash
source ~/PiperSim/start_sim.sh    # 仿真环境（Mock / Gazebo）
source ~/PiperSim/start_real.sh   # 真机环境
```

脚本自动完成：
1. 激活 conda 环境 `piper_sdk`
2. Source ROS 2 Humble 环境（fishros_humble）
3. Source PiperSim 工作空间

**必须用 `source` 命令运行**，否则环境不会在当前 shell 生效。

## 环境说明
- **conda 环境**: `piper_sdk`（包含项目所需的所有 Python 依赖）
- **ROS 2 环境**: `fishros_humble`（ROS 2 Humble 发行版）
- 由于在 `~/.bashrc` 中禁用了 ROS 2 自动启动，需要手动 source

## 启动模式
| 模式 | 命令 |
|---|---|
| Mock+MoveIt(无硬件,推荐) | `source start_sim.sh && ros2 launch piper_moveit_config demo.launch.xml` |
| Gazebo+MoveIt | `source start_sim.sh && ros2 launch piper_moveit_config demo.launch.xml sim_gazebo:=true` |
| 真机 | `source start_real.sh && ros2 launch piper_moveit_config demo.launch.xml real_hardware:=true` |

## xacro 组装链
`piper.urdf.xacro` → `load_piper`(piper_macro) → `piper_arm_macro` + `piper_gripper_macro` + `piper.ros2_control.xacro`(+ `piper.gazebo.xacro`)。硬件后端由 xacro 参数切换:`mock_hardware` / `sim_gazebo` / `real_hardware`。

## ⚠️ 关节命名(必读,曾因此出 bug)
真值 = `piper.ros2_control.xacro`,恰好 **7 个指令关节**:
- 手臂 `joint1`..`joint6`
- 夹爪(左指,官方模型的 joint7)= **`gripper_joint`**
- 右指(官方 joint8)= **`right_finger_joint`**,URDF `<mimic joint="gripper_joint" multiplier="-1">`

历史坑:commit 4091c00 重命名后,C++/controllers/脚本曾残留旧名 `joint7`/`joint8`。**新增任何 controller yaml / SRDF / 脚本必须用上面的名字。**

## 硬件接口(`piper_hardware.cpp` / `.hpp`)
- `NUM_JOINTS=7`(6 臂 + gripper_joint);right_finger 是 mimic,不导出接口。
- 索引:`hw_cmd_[0..5]`=joint1-6,`hw_cmd_[6]`=夹爪;仅导出 `position`。
- 单线程 `read()`→`write()`;CAN 读用 `drain_can_rx()`(持锁跨 select+read)。
- CAN(piper_sdk V2)速查(完整表见 README):使能 `0x471`,模式 `0x151`(CAN+MOVEJ),关节指令 `0x155/156/157`(2×int32 大端,0.001°),夹爪 `0x159`;反馈 `0x2A5/6/7`(关节)、`0x2A8`(夹爪,状态字节含过热/错误位)。换算 `RAD_TO_MDEG=57295.7795`、`METER_TO_UMM=1e6`。

## 运动学真值
- **官方 MuJoCo 模型**:`/home/dream/my_robot_models/agilex_piper/piper.xml`(网格在 `assets/`)。
- 仓库 `meshes/*.stl` 与官方 **md5 一致** → 官方 `pos`/`quat` 变换可直接套用。
- URDF 用 rpy(Rz·Ry·Rx);校验时把官方 quat 转 rpy 做 FK 逐帧对比(误差应 <1mm/0.5°)。
- 零位准确;**joint3 只能负向 `[-2.697, 0]`**(≈ -180°~0°)。
- 夹爪:左右指同挂 `link6 + (0,0,0.13503)`,镜像姿态,耦合 `joint8=-joint7`;`link6` 视觉绕 **Z** +90°(不是 X)。

## 关节限位(rad)
`joint1 ±2.618` · `joint2 0~3.14` · `joint3 -2.697~0` · `joint4 ±1.832` · `joint5 ±1.22` · `joint6 ±π` · `gripper 0~0.035`

## 构建 & 验证
```bash
cd ~/PiperSim && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
xacro src/piper_description/urdf/piper.urdf.xacro real_hardware:=true   # URDF 生成自检
```

## 已修复(勿重复修)
- 硬件 `NUM_JOINTS` 8→7(export_state_interfaces 越界崩溃);`piper_controllers.yaml` 夹爪 `joint7`→`gripper_joint`
- `drain_can_rx` select/read 竞态;删除死代码 `recv_can_frame`
- 模型装配:`link6` 视觉朝向(X→Z);夹爪按官方重建(去掉伪造方块基座,修正位置/滑动轴/mimic 系数)
- `moveit_controllers.yaml` 漏注册 `gripper_controller` → 补上 `gripper_joint` 的 FollowJointTrajectory 项(否则 SRDF `gripper` group plan and execute 报"no controller")
- joint3 限位跨文件统一为 `-2.697` / `0`(arm_macro `−3.0` / ros2_control `−2.967` → 全部 `−2.697`)
- **MoveIt Mock 模式修复(2026-07)**:
  - `ompl_planning.yaml`: 修复 `request_adapters` 参数类型(数组→字符串),解决 move_group 崩溃
  - `kinematics.yaml`: 增加 timeout(0.05s→5s),移除错误的 gripper 运动学配置
  - `moveit.rviz`: Fixed Frame 从 `table` 改为 `base_link`
  - `demo.launch.xml`: 添加显式的 `robot_description` 参数
  - `mock_controllers.yaml`: 修复控制器配置格式(匹配 `piper_controllers.yaml`)
  - `mock_bringup.launch.py`: 增加控制器启动延迟(2s/4s/6s)
  - `piper.srdf.xacro`: 添加完整的碰撞禁用配置,`base_link` 与 `table` 碰撞设为 `Never`
  - 新增 `start_sim.sh` / `start_real.sh`: 一键 source 脚本

## 路线图
见 README「开发路线」:MuJoCo 后端、仿真场景、sim2real、自主数据采集、世界模型。