#!/usr/bin/env python3
"""
MuJoCo仿真控制真机脚本

功能：
1. 订阅 MuJoCo 仿真的 /joint_states
2. 实时发送到真机控制器
3. 支持安全限制和紧急停止

使用方法：
    # Terminal 1: 启动MuJoCo仿真（手动控制模式）
    ros2 launch piper_mujoco mujoco_manual_control.launch.py

    # Terminal 2: 启动真机
    source ~/PiperSim/start_real.sh
    ros2 launch piper_moveit_config demo.launch.xml real_hardware:=true

    # Terminal 3: 运行同步脚本
    python3 src/piper_mujoco/scripts/sim_to_real_sync.py
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
import time
from threading import Thread
import sys


class SimToRealSync(Node):
    """仿真到真机同步节点"""

    def __init__(self):
        super().__init__('sim_to_real_sync')

        # 参数
        self.declare_parameter('control_frequency', 50.0)  # Hz
        self.declare_parameter('joint_names', 
            ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('gripper_joint', 'gripper_joint')
        self.declare_parameter('safe_mode', True)
        self.declare_parameter('max_velocity', 1.0)  # rad/s

        self.control_freq = self.get_parameter('control_frequency').value
        self.joint_names = self.get_parameter('joint_names').value
        self.gripper_joint = self.get_parameter('gripper_joint').value
        self.safe_mode = self.get_parameter('safe_mode').value
        self.max_velocity = self.get_parameter('max_velocity').value

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

        # 发布真机轨迹命令
        self.real_arm_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.real_gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
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
        self.last_positions = {}
        self.estop_active = False
        self.last_update_time = time.time()

        # 控制线程
        self.control_thread = Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.get_logger().info('Sim to Real Sync Node Started')
        self.get_logger().info(f'Control frequency: {self.control_freq} Hz')
        self.get_logger().info(f'Safe mode: {self.safe_mode}')

    def sim_joint_callback(self, msg):
        """接收仿真关节状态"""
        if self.estop_active:
            return

        # 更新当前仿真位置
        for i, name in enumerate(msg.name):
            if name in self.joint_names or name == self.gripper_joint:
                self.current_sim_positions[name] = msg.position[i]

    def estop_callback(self, msg):
        """紧急停止回调"""
        if msg.data:
            self.estop_active = True
            self.get_logger().error('Emergency Stop Activated!')
        else:
            self.estop_active = False
            self.get_logger().info('Emergency Stop Released')

    def check_safety(self, new_positions):
        """安全检查"""
        if not self.safe_mode:
            return True

        current_time = time.time()
        dt = current_time - self.last_update_time

        if dt <= 0:
            return False

        # 检查速度限制
        for joint_name, position in new_positions.items():
            if joint_name in self.last_positions:
                velocity = abs(position - self.last_positions[joint_name]) / dt
                if velocity > self.max_velocity:
                    self.get_logger().warn(
                        f'Joint {joint_name} velocity too high: {velocity:.2f} rad/s '
                        f'(max: {self.max_velocity} rad/s)'
                    )
                    return False

        return True

    def control_loop(self):
        """控制循环"""
        rate = 1.0 / self.control_freq

        while rclpy.ok():
            if self.estop_active or not self.current_sim_positions:
                time.sleep(rate)
                continue

            # 发送命令到真机
            self.send_to_real_robot()

            # 更新时间戳
            self.last_update_time = time.time()
            self.last_positions = self.current_sim_positions.copy()

            time.sleep(rate)

    def send_to_real_robot(self):
        """发送命令到真机"""
        if len(self.current_sim_positions) < 7:
            return

        # 安全检查
        if not self.check_safety(self.current_sim_positions):
            return

        # 创建机械臂轨迹消息
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.joint_names

        arm_point = JointTrajectoryPoint()
        for joint_name in self.joint_names:
            if joint_name in self.current_sim_positions:
                arm_point.positions.append(self.current_sim_positions[joint_name])

        arm_point.time_from_start.sec = 0
        arm_point.time_from_start.nanosec = int(1.0 / self.control_freq * 1e9)
        arm_trajectory.points.append(arm_point)

        # 创建夹爪轨迹消息
        gripper_trajectory = JointTrajectory()
        gripper_trajectory.joint_names = [self.gripper_joint]

        gripper_point = JointTrajectoryPoint()
        if self.gripper_joint in self.current_sim_positions:
            gripper_point.positions.append(self.current_sim_positions[self.gripper_joint])

        gripper_point.time_from_start.sec = 0
        gripper_point.time_from_start.nanosec = int(1.0 / self.control_freq * 1e9)
        gripper_trajectory.points.append(gripper_point)

        # 发布命令
        self.real_arm_pub.publish(arm_trajectory)
        self.real_gripper_pub.publish(gripper_trajectory)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimToRealSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()