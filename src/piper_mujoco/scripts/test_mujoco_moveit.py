#!/usr/bin/env python3
"""
MuJoCo 仿真测试脚本
测试 MoveIt 运动规划 + MuJoCo 仿真

用法：
  终端 1: source install/setup.bash && ros2 launch piper_moveit_config demo.launch.xml sim_mujoco:=true
  终端 2: python3 src/piper_mujoco/scripts/test_mujoco_moveit.py

注意：当前版本使用 mock 硬件接口，MuJoCo 物理仿真待实现
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    WorkspaceParameters
)
import time
import threading
import math


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class MuJoCoMoveItTester(Node):
    """MuJoCo + MoveIt 测试节点"""

    def __init__(self):
        super().__init__("mujoco_moveit_tester")
        self._current_joints = None

        # 订阅关节状态
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

        # MoveIt MoveGroup action client
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("MuJoCo MoveIt Tester initialized")

    def _joint_cb(self, msg):
        d = {}
        for name, pos in zip(msg.name, msg.position):
            d[name] = pos
        self._current_joints = d

    def get_current_q(self):
        if self._current_joints is None:
            return None
        return [self._current_joints.get(n, 0.0) for n in JOINT_NAMES]

    def move_to_joints(self, joint_positions, planning_time=5.0):
        """使用 MoveIt 移动到目标关节角度"""
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt MoveGroup server not available!")
            self.get_logger().error("Make sure MoveIt is running: ros2 launch piper_moveit_config demo.launch.xml")
            return False

        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        constraints.name = "move_to_joints"

        for name, pos in zip(JOINT_NAMES, joint_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = pos
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        request.goal_constraints.append(constraints)

        workspace = WorkspaceParameters()
        workspace.header.frame_id = "base_link"
        workspace.min_corner.x = -1.0
        workspace.min_corner.y = -1.0
        workspace.min_corner.z = -0.1
        workspace.max_corner.x = 1.0
        workspace.max_corner.y = 1.0
        workspace.max_corner.z = 1.0
        request.workspace_parameters = workspace

        goal.request = request
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(f"Sending joint goal: {[f'{p:.3f}' for p in joint_positions]}")
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info("Motion completed successfully!")
            return True
        else:
            self.get_logger().error(f"Motion failed with error code: {result.result.error_code.val}")
            return False


def print_banner():
    print("\n" + "="*60)
    print("   MuJoCo + MoveIt 仿真测试")
    print("="*60)
    print("\n测试项目：")
    print("  1. 回零位")
    print("  2. 移动到预设位置 1（前伸）")
    print("  3. 移动到预设位置 2（侧伸）")
    print("  4. 移动到预设位置 3（上方）")
    print("  5. 移动到笛卡尔位置")
    print("  6. 自定义关节角度")
    print("  q. 退出")
    print("\n")


def main():
    rclpy.init()
    node = MuJoCoMoveItTester()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("Waiting for MoveIt server...")
    time.sleep(3)

    poses = {
        "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "forward": [0.0, 1.0, -0.85, 0.0, 0.85, 0.0],
        "side": [1.57, 0.5, -0.5, 0.0, 0.5, 0.0],
        "up": [0.0, 0.3, -0.3, 0.0, 1.0, 0.0],
    }

    print_banner()

    try:
        while True:
            q = node.get_current_q()
            if q is not None:
                print(f"\n当前关节: [{', '.join(f'{j:.3f}' for j in q)}]")

            choice = input("\n请选择测试项目 (1-6, q): ").strip()

            if choice == 'q':
                break
            elif choice == '1':
                print("\n[测试 1] 回零位...")
                node.move_to_joints(poses["home"])
            elif choice == '2':
                print("\n[测试 2] 移动到前伸位置...")
                node.move_to_joints(poses["forward"])
            elif choice == '3':
                print("\n[测试 3] 移动到侧伸位置...")
                node.move_to_joints(poses["side"])
            elif choice == '4':
                print("\n[测试 4] 移动到上方位置...")
                node.move_to_joints(poses["up"])
            elif choice == '5':
                print("\n[测试 5] 移动到笛卡尔位置...")
                x = float(input("  X (m): ") or "0.3")
                y = float(input("  Y (m): ") or "0.0")
                z = float(input("  Z (m): ") or "0.3")
                # TODO: 实现位姿规划
                print("  位姿规划功能待实现")
            elif choice == '6':
                print("\n[测试 6] 自定义关节角度...")
                print("  输入 6 个关节角度（弧度），用空格分隔")
                user_input = input("  关节角度: ").strip()
                try:
                    angles = [float(x) for x in user_input.split()]
                    if len(angles) != 6:
                        print("  ERROR: 需要 6 个角度")
                        continue
                    node.move_to_joints(angles)
                except ValueError:
                    print("  ERROR: 输入格式错误")
            else:
                print("  无效选择")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Bye!")


if __name__ == "__main__":
    main()