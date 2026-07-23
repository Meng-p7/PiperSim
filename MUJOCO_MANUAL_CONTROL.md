# MuJoCo 手动控制说明

## 重要概念

### MuJoCo 的控制机制

MuJoCo 使用 **actuator** 来控制关节：

1. **Position Actuator**: 设置目标位置，actuator会自动控制关节到目标
2. **GUI滑块**: 控制actuator的控制目标（ctrl），不是直接控制关节位置

### ros2_control 的影响

当启动 `mujoco_ros2_control_node` 时：
- ROS 2 控制器会向 actuator 发送命令
- 如果同时用 GUI 滑块，会产生冲突
- 需要停止 ROS 2 控制器，GUI 滑块才能工作

---

## 解决方案

### 方案 1: 手动控制模式（推荐）

**启动命令**:
```bash
ros2 launch piper_mujoco mujoco_manual_control.launch.py
```

**工作原理**:
- 只启动 `joint_state_broadcaster`
- 不启动 trajectory controller
- GUI 滑块可以控制 actuator

**使用步骤**:
1. 等待 MuJoCo GUI 打开
2. 按 **]** 键显示右侧控制面板
3. 找到 **actuation** 或 **Joint** 面板
4. **拖动滑块** - 这会修改 actuator 的控制目标
5. 观察机器人运动到目标位置

**特点**:
- ✅ 可以在 GUI 中控制
- ✅ joint_states 仍然发布到 ROS 2
- ⚠️ 滑块控制的是位置目标，有延迟
- ⚠️ 不是直接拖动关节

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

**特点**:
- ✅ 完整的 MoveIt 集成
- ✅ Plan & Execute 功能
- ❌ GUI 滑块无法使用（控制器优先级更高）

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

**特点**:
- ✅ 完全手动控制
- ✅ GUI 滑块完全可用
- ❌ 没有 ROS 2 集成
- ❌ 不能用 MoveIt

---

## 常见问题

### Q1: 为什么滑块拖动后机器人不动？

**原因**: MuJoCo 的 position actuator 有延迟，需要等待它控制关节到目标位置。

**解决**:
- 等待几秒钟
- 或者使用更小的 kp/kv 参数（在 piper.xml 中）

### Q2: 能否直接拖动关节（像 Gazebo）？

**不能**。MuJoCo 的 actuator 是控制器，不能直接拖动。

**替代方案**:
- 使用滑块设置目标位置
- 或使用纯 MuJoCo 模式（方案3）

### Q3: 手动控制模式下 RViz 能看到吗？

**能**。joint_state_broadcaster 会发布状态，RViz 可以订阅。

### Q4: 标准模式下为什么滑块不能动？

**原因**: ROS 2 控制器优先级高于 GUI 输入。

**解决**: 使用手动控制模式（方案1）

---

## 推荐使用场景

| 场景 | 推荐方案 |
|------|---------|
| 快速开发 | Mock 模式 |
| 轨迹规划测试 | Sim 模式（方案2） |
| 手动调试关节 | 手动控制（方案1） |
| 纯物理仿真 | 纯 MuJoCo（方案3） |
| 真机控制 | Real 模式 |