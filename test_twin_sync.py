#!/usr/bin/env python3
"""
测试脚本：验证真机状态是否正确发布
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time


class TestTwinSync(Node):
    """测试数字孪生同步"""

    def __init__(self):
        super().__init__('test_twin_sync')

        # 订阅真机状态
        self.real_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.real_callback,
            10
        )

        # 订阅MuJoCo轨迹（验证是否有数据）
        self.mujoco_sub = self.create_subscription(
            JointTrajectory,
            '/mujoco_joint_trajectory_controller/joint_trajectory',
            self.mujoco_callback,
            10
        )

        # 发布到MuJoCo
        self.mujoco_pub = self.create_publisher(
            JointTrajectory,
            '/mujoco_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.real_count = 0
        self.mujoco_count = 0
        self.last_positions = None

        self.get_logger().info('=== Testing Twin Sync ===')
        self.get_logger().info('Subscribing to /joint_states (real robot)')
        self.get_logger().info('Subscribing to /mujoco_joint_trajectory_controller/joint_trajectory')
        self.get_logger().info('Waiting for data...')

    def real_callback(self, msg):
        """接收真机状态"""
        self.real_count += 1
        self.last_positions = msg.position[:6]  # 只取前6个关节
        
        if self.real_count <= 3 or self.real_count % 30 == 0:
            self.get_logger().info(f'[REAL] Received #{self.real_count}: positions={self.last_positions[:3]}')

        # 立即发布到MuJoCo（测试）
        if self.real_count <= 10:
            trajectory = JointTrajectory()
            trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            trajectory.points = [{
                'positions': self.last_positions,
                'time_from_start': {'sec': 0, 'nanosec': 33000000}
            }]
            self.mujoco_pub.publish(trajectory)

    def mujoco_callback(self, msg):
        """接收MuJoCo轨迹"""
        self.mujoco_count += 1
        if self.mujoco_count <= 3:
            self.get_logger().info(f'[MUJOCO] Received trajectory #{self.mujoco_count}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TestTwinSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()