#!/usr/bin/env python3
"""
数字孪生同步脚本 - 实时版本

功能：将真机位置实时同步到MuJoCo
方法：订阅真机joint_states，发送单点轨迹到MuJoCo控制器
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class DigitalTwinSync(Node):
    """数字孪生同步节点"""

    def __init__(self):
        super().__init__('digital_twin_sync')

        # 参数
        self.declare_parameter('sync_frequency', 30.0)  # Hz
        self.declare_parameter('joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])

        self.sync_freq = self.get_parameter('sync_frequency').value
        self.joint_names = self.get_parameter('joint_names').value

        # 订阅真机关节状态
        self.real_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.real_joint_callback,
            10
        )

        # 发布轨迹到MuJoCo（直接话题方式）
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/mujoco_joint_trajectory_controller/joint_trajectory',  # MuJoCo专用话题
            10
        )

        # 状态变量
        self.current_positions = None
        self.first_state_received = False
        self.last_send_time = 0.0

        # 定时器：定期发送轨迹
        self.timer = self.create_timer(1.0 / self.sync_freq, self.send_trajectory)

        self.get_logger().info('Digital Twin Sync Started')
        self.get_logger().info(f'Sync frequency: {self.sync_freq} Hz')
        self.get_logger().info('Waiting for real robot joint states...')

    def real_joint_callback(self, msg):
        """接收真机关节状态"""
        # 提取前6个关节的位置（忽略夹爪）
        positions = []
        for joint_name in self.joint_names:
            try:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            except (ValueError, IndexError):
                self.get_logger().warn(f'Joint {joint_name} not found in message')
                return

        self.current_positions = positions

        if not self.first_state_received:
            self.get_logger().info('✓ Real robot state received')
            self.get_logger().info('Starting synchronization to MuJoCo...')
            self.first_state_received = True

    def send_trajectory(self):
        """发送单点轨迹到MuJoCo"""
        if not self.current_positions:
            return

        # 创建轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # 创建单点轨迹（立即到达）
        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 33_000_000  # 约30Hz
        trajectory.points.append(point)

        # 发布轨迹
        self.trajectory_pub.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DigitalTwinSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()