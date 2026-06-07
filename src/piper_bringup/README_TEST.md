# Gazebo 仿真 + MoveIt 测试指南

## 前置条件

确保安装了必要的依赖：
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control
```

## 测试步骤

### 1. 启动 Gazebo 仿真 + MoveIt

**终端 1**：启动 Gazebo 仿真
```bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_bringup gazebo_piper.launch.py
```

等待 Gazebo 启动完成（看到 `Ignition Gazebo Server` 字样）

**终端 2**：启动 MoveIt
```bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_moveit_config demo.launch.xml use_sim_time:=true
```

等待 MoveIt 启动完成（看到 `MoveGroup action ready` 字样）

### 2. 运行测试脚本

**终端 3**：运行测试脚本
```bash
source ~/PiperSim/install/setup.bash
python3 src/piper_bringup/scripts/test_gazebo_moveit.py
```

### 3. 测试项目

脚本提供 6 个测试项目：

1. **回零位** - 移动到初始位置
2. **前伸位置** - 机械臂向前伸展
3. **侧伸位置** - 机械臂向侧面伸展
4. **上方位置** - 机械臂向上抬起
5. **笛卡尔位置** - 输入 XYZ 坐标移动
6. **自定义关节角度** - 输入 6 个关节角度

### 4. 观察结果

- 在 Gazebo 中观察机械臂运动
- 在 RViz 中观察 MoveIt 规划轨迹
- 终端会显示运动状态和误差

---

## 真机验证标定（MoveIt 版本）

### 启动步骤

**终端 1**：启动真机控制
```bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_control real_bringup.launch.py can:=can0
```

**终端 2**：启动 MoveIt（真机模式）
```bash
source ~/PiperSim/install/setup.bash
ros2 launch piper_moveit_config moveit_demo.launch.py
```

**终端 3**：运行验证脚本
```bash
source ~/PiperSim/install/setup.bash
python3 src/piper_calibration/piper_calibration/verify_calibration_moveit.py \
  --result-file data/real_eye_in_hand_result.yaml \
  --tag-id 1 \
  --tag-size 0.057
```

### 操作说明

1. 将 AprilTag 标定板放在相机视野内
2. 相机窗口会显示检测到的标定板
3. 按 **空格键** - 机械臂移动到标定板上方 10cm
4. 按 **Q** 或 **ESC** - 退出

---

## 常见问题

### 1. MoveIt 服务器连接失败

```
ERROR: MoveIt MoveGroup server not available!
```

**解决**：确保 MoveIt 已启动，等待几秒后重试

### 2. Gazebo 插件加载失败

```
Failed to load system plugin [gz_ros2_control-system]
```

**解决**：安装 gz_ros2_control
```bash
sudo apt install ros-humble-gz-ros2-control
```

### 3. 关节状态不更新

**解决**：检查 controller_manager 是否正常
```bash
ros2 control list_controllers
```

### 4. 运动规划失败

**解决**：
- 检查目标位置是否在工作空间内
- 增加规划时间：修改 `planning_time` 参数
- 检查关节限位设置

---

## 文件说明

- `scripts/test_gazebo_moveit.py` - Gazebo 仿真测试脚本
- `scripts/verify_calibration_moveit.py` - 真机标定验证脚本（MoveIt 版本）
- `scripts/verify_calibration.py` - 真机标定验证脚本（原版，使用 MuJoCo IK）
