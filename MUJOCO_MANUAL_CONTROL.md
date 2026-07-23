# MuJoCo 控制说明


### 方案 1: 手动控制模式

**启动命令**:
```bash
ros2 launch piper_mujoco mujoco_manual_control.launch.py
```

**工作原理**:
- 只启动 `joint_state_broadcaster`
- 不启动 trajectory controller
- GUI 滑块可以控制 actuator

---

### 方案 2: 标准仿真模式

**启动命令**:
```bash
ros2 launch piper_moveit_config demo.launch.py mode:=sim
```

**工作原理**:
- 启动所有 ROS 2 控制器
- MoveIt 规划轨迹
- MuJoCo 执行轨迹

---

### 方案 3: 纯 MuJoCo 仿真（无 ROS 2）

**启动命令**:
```bash
# 直接运行 MuJoCo（不通过 ROS 2）
simulate ~/PiperSim/src/piper_mujoco/models/piper.xml
```

**工作原理**:
- 纯 MuJoCo 仿真
- 不通过 ROS 2
- 完全手动控制