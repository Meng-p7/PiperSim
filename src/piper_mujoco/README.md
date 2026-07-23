# Piper MuJoCo 仿真包

基于 MuJoCo 物理引擎的 Piper 机械臂仿真包。

## 当前状态

 **已完全集成并测试通过**

MuJoCo 物理仿真已成功集成到 PiperSim 项目中，支持：
-  MuJoCo 3.4.0 物理引擎
-  MoveIt 轨迹规划
-  实时物理仿真
-  关节状态实时发布
-  夹爪联动控制


### 禁用GUI窗口（可选）
如果不需要可视化窗口，可以禁用：
```bash
ros2 launch piper_moveit_config demo.launch.xml sim_mujoco:=true headless:=true
```

## 集成细节

### 关键问题解决

**问题1：模型路径解析**
- ✅ CMakeLists.txt 正确安装 models 目录
- ✅ `$(find piper_mujoco)` 正确解析为安装路径

**问题2：关节名称匹配**
- ✅ 修改 MuJoCo 模型：joint7→gripper_joint, joint8→right_finger_joint
- ✅ 确保与 URDF 命名一致
- ✅ actuator 映射正确配置

**问题3：硬件接口**
- ✅ 使用 `MujocoSystemInterface` 插件
- ✅ 所有关节注册成功
- ✅ 控制器全部 active

## 使用方法

### 方式一：通过 MoveIt 启动

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_moveit_config demo.launch.xml sim_mujoco:=true
```

### 方式二：单独启动 MuJoCo 仿真

```bash
source ~/PiperSim/start_sim.sh
ros2 launch piper_mujoco mujoco_piper.launch.py
```

## 测试

```bash
python3 src/piper_mujoco/scripts/test_mujoco_moveit.py
```

## 未来优化方向

1. **性能优化**
   - 实时性调优（控制频率优化）
   - 多实例并行仿真

2. **传感器仿真**
   - 相机仿真（RGB-D）
   - 力/力矩传感器
   - 接触传感器

3. **场景扩展**
   - 乒乓球桌、球拍等物体模型
   - 物理属性配置（摩擦、弹性等）
   - 碰撞检测优化

4. **强化学习集成**
   - Gym 环境接口
   - 并行环境支持
   - 域随机化

## MuJoCo 模型

MuJoCo 模型文件已包含在项目中：
- `models/piper.xml` - MuJoCo 机器人模型（15KB）
- `models/assets/` - 网格文件（STL/OBJ，约32MB）
- `models/MODEL_LICENSE` - 模型版权信息

模型来源于官方 [Agilex Piper MuJoCo 模型库](https://github.com/google-deepmind/mujoco_menagerie)。

**注意**：克隆本项目后，所有MuJoCo模型文件都已包含在内，无需额外下载。

## 目录结构

```
piper_mujoco/
├── launch/
│   └── mujoco_piper.launch.py    # MuJoCo 启动文件
├── config/
│   └── mujoco_controllers.yaml    # 控制器配置
├── scripts/
│   └── test_mujoco_moveit.py      # 测试脚本
└── models/                         # MuJoCo 模型（约32MB）
    ├── piper.xml                   # 机器人模型
    ├── MODEL_LICENSE               # 版权信息
    └── assets/                     # 网格文件（STL/OBJ）
```

## 依赖

```bash
# MuJoCo Python 包
pip install mujoco

# MuJoCo ROS 2 集成包（必需）
sudo apt install ros-humble-mujoco-ros2-control ros-humble-mujoco-ros2-control-demos

# ROS 2 相关
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller
```

## 许可证

MIT