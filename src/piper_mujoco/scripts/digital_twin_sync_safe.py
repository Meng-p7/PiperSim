#!/usr/bin/env python3
"""
数字孪生同步脚本（安全版本）- 修复版

功能：Real -> MuJoCo 单向同步

工作原理：
1. 订阅真机的 /joint_states (从真机控制器)
2. 发布到 MuJoCo 的 /mujoco/joint_states (MuJoCo会读取)
3. MuJoCo模型实时跟随真机
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class DigitalTwinSync(Node):
    """数字孪生同步节点（仅 Real -> MuJoCo）"""

    def __init__(self):
        super().__init__('digital_twin_sync')

        # 参数
        self.declare_parameter('sync_frequency', 50.0)  # Hz
        self.declare_parameter('joint_names',
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joint', 'gripper_joint')

        self.sync_freq = self.get_parameter('sync_frequency').value
        self.joint_names = self.get_parameter('joint_names').value
        self.gripper_joint = self.get_parameter('gripper_joint').value

        # 订阅真机关节状态
        self.real_joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.real_joint_callback,
            10
        )

        # 发布到MuJoCo的joint_states（让MuJoCo读取）
        self.mujoco_pub = self.create_publisher(
            JointState,
            '/mujoco_joint_states',
            10
        )

        # 状态变量
        self.current_real_state = None
        self.first_state_received = False

        # 定时器：定期发布到MuJoCo
        self.timer = self.create_timer(1.0 / self.sync_freq, self.publish_to_mujoco)

        self.get_logger().info('Digital Twin Sync Started')
        self.get_logger().info('Mode: Real -> MuJoCo (Safety Mode)')
        self.get_logger().warn('MuJoCo -> Real sync is DISABLED for safety!')
        self.get_logger().info('Sync frequency: {} Hz'.format(self.sync_freq))

    def real_joint_callback(self, msg):
        """
        接收真机关节状态
        """
        self.current_real_state = msg
        
        if not self.first_state_received:
            self.get_logger().info('Real robot state received. MuJoCo will mirror real robot position.')
            self.first_state_received = True

    def publish_to_mujoco(self):
        """
        定期发布真机状态到MuJoCo
        """
        if not self.current_real_state:
            return
        
        # 直接转发真机的joint_states到MuJoCo
        self.mujoco_pub.publish(self.current_real_state)


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