#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointStatePublisher(Node):
    def __init__(self):
        super().__init__("static_joint_state_publisher")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.05, self.publish)
        self.msg = JointState()
        self.msg.name = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6",
            "joint7", "joint8",
        ]
        self.msg.position = [0.0, 1.57, -1.3485, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg.velocity = [0.0] * 8
        self.msg.effort = [0.0] * 8

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = StaticJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
