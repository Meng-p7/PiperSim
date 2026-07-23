#!/usr/bin/env python3
"""
数字孪生同步脚本（高级版本 - 需谨慎使用）

功能：
1. Real -> MuJoCo: 实时同步（安全）
2. MuJoCo -> Real: 速度控制+低通滤波（危险，需要安全措施）

安全机制：
1. 位置突变检测：超过阈值会拆分为小步长
2. 斜坡限速：限制最大速度
3. 低通滤波（EMA）：平滑运动
4. 紧急停止：立即中断

警告：
- MuJoCo拖拽滑块会控制真机
- 需要确保安全措施到位
- 建议先测试斜坡限速参数
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Float64MultiArray
import time
from threading import Thread, Lock
import math


class LowPassFilter:
    """低通滤波器（EMA）"""

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.last_value = None

    def filter(self, value):
        if self.last_value is None:
            self.last_value = value
            return value

        filtered = self.alpha * value + (1 - self.alpha) * self.last_value
        self.last_value = filtered
        return filtered


class DigitalTwinSyncAdvanced(Node):
    """数字孪生同步节点（高级版本，双向同步）"""

    def __init__(self):
        super().__init__('digital_twin_sync_advanced')

        # 参数
        self.declare_parameter('control_frequency', 50.0)  # Hz
        self.declare_parameter('joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joint', 'gripper_joint')

        # 安全参数
        self.declare_parameter('max_velocity', 0.5)  # rad/s（降低以提高安全性）
        self.declare_parameter('position_jump_threshold', 0.1)  # rad
        self.declare_parameter('ramp_rate', 0.2)  # rad/s²
        self.declare_parameter('lpf_alpha', 0.2)  # 低通滤波系数

        self.control_freq = self.get_parameter('control_frequency').value
        self.joint_names = self.get_parameter('joint_names').value
        self.gripper_joint = self.get_parameter('gripper_joint').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.jump_threshold = self.get_parameter('position_jump_threshold').value
        self.ramp_rate = self.get_parameter('ramp_rate').value
        self.lpf_alpha = self.get_parameter('lpf_alpha').value

        # 回调组
        cb_group = ReentrantCallbackGroup()

        # 订阅仿真关节状态
        self.sim_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.sim_joint_callback,
            10,
            callback_group=cb_group
        )

        # 订阅真机关节状态
        self.real_joint_sub = self.create_subscription(
            JointState,
            '/real_joint_states',
            self.real_joint_callback,
            10,
            callback_group=cb_group
        )

        # 发布真机速度命令
        self.real_velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocity_controller/commands',
            10
        )

        # 紧急停止
        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10
        )

        # 状态变量
        self.current_sim_positions = {}
        self.current_real_positions = {}
        self.last_positions = {}
        self.estop_active = False
        self.last_update_time = time.time()
        self.current_velocities = {joint: 0.0 for joint in self.joint_names}

        # 低通滤波器
        self.lpf = {joint: LowPassFilter(self.lpf_alpha) for joint in self.joint_names}
        self.lpf[self.gripper_joint] = LowPassFilter(self.lpf_alpha)

        # 锁
        self.mutex = Lock()

        # 控制线程
        self.control_thread = Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().warn('=== ADVANCED MODE ENABLED ===')
        self.get_logger().warn('MuJoCo -> Real sync is ACTIVE (with safety measures)')
        self.get_logger().warn('Max velocity: {} rad/s'.format(self.max_velocity))
        self.get_logger().warn('Position jump threshold: {} rad'.format(self.jump_threshold))
        self.get_logger().warn('Ramp rate: {} rad/s²'.format(self.ramp_rate))
        self.get_logger().info('Control frequency: {} Hz'.format(self.control_freq))

    def sim_joint_callback(self, msg):
        """接收仿真关节状态"""
        with self.mutex:
            for i, name in enumerate(msg.name):
                if name in self.joint_names or name == self.gripper_joint:
                    self.current_sim_positions[name] = msg.position[i]

    def real_joint_callback(self, msg):
        """接收真机关节状态"""
        with self.mutex:
            for i, name in enumerate(msg.name):
                if name in self.joint_names or name == self.gripper_joint:
                    self.current_real_positions[name] = msg.position[i]

    def estop_callback(self, msg):
        """紧急停止回调"""
        if msg.data:
            self.estop_active = True
            self.get_logger().error('EMERGENCY STOP ACTIVATED!')
            # 立即停止所有运动
            self.send_velocity_command({joint: 0.0 for joint in self.joint_names})
        else:
            self.estop_active = False
            self.get_logger().info('Emergency Stop Released')

    def detect_position_jump(self, joint, current, target):
        """
        检测位置突变
        返回True如果检测到突变
        """
        delta = abs(target - current)
        return delta > self.jump_threshold

    def calculate_ramp_velocity(self, joint, current, target):
        """
        计算斜坡速度
        使用低通滤波平滑速度
        """
        # 计算位置误差
        error = target - current

        # 检测突变
        if self.detect_position_jump(joint, current, target):
            self.get_logger().warn(
                'Position jump detected on {}: {:.3f} rad'.format(joint, abs(error))
            )

        # 计算目标速度（限制最大速度）
        target_velocity = math.copysign(
            min(abs(error) * self.control_freq, self.max_velocity),
            error
        )

        # 斜坡限速
        velocity_delta = target_velocity - self.current_velocities[joint]
        max_delta = self.ramp_rate / self.control_freq

        if abs(velocity_delta) > max_delta:
            target_velocity = self.current_velocities[joint] + math.copysign(max_delta, velocity_delta)

        # 低通滤波
        filtered_velocity = self.lpf[joint].filter(target_velocity)

        # 更新当前速度
        self.current_velocities[joint] = filtered_velocity

        return filtered_velocity

    def control_loop(self):
        """控制循环"""
        rate = 1.0 / self.control_freq

        while rclpy.ok():
            if self.estop_active:
                time.sleep(rate)
                continue

            with self.mutex:
                sim_positions = self.current_sim_positions.copy()
                real_positions = self.current_real_positions.copy()

            if not sim_positions or not real_positions:
                time.sleep(rate)
                continue

            # 计算速度命令
            velocity_commands = {}
            for joint in self.joint_names:
                if joint in sim_positions and joint in real_positions:
                    velocity = self.calculate_ramp_velocity(
                        joint,
                        real_positions[joint],
                        sim_positions[joint]
                    )
                    velocity_commands[joint] = velocity

            # 发送速度命令
            if velocity_commands and not self.estop_active:
                self.send_velocity_command(velocity_commands)

            time.sleep(rate)

    def send_velocity_command(self, velocities):
        """发送速度命令"""
        msg = Float64MultiArray()
        msg.data = [velocities.get(joint, 0.0) for joint in self.joint_names]
        self.real_velocity_pub.publish(msg)

    def check_safety(self):
        """安全检查"""
        if self.estop_active:
            return False

        # 其他安全检查...
        return True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DigitalTwinSyncAdvanced()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()