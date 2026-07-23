#!/usr/bin/env python3
"""
测试真机轨迹执行
发送一个简单的轨迹命令到 joint_trajectory_controller
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import time

class TrajectoryTest(Node):
    def __init__(self):
        super().__init__('trajectory_test')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]

        # 简单轨迹：从当前位置移动到零位
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 3
        goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        self.get_logger().info('Sending trajectory goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f'Goal accepted: {future.result().accepted}')
        else:
            self.get_logger().error('Goal failed')

def main():
    rclpy.init()
    test = TrajectoryTest()
    test.send_goal()
    test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()