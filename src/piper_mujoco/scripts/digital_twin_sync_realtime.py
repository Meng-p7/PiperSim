#!/usr/bin/env python3
"""
数字孪生同步脚本 - 直接MuJoCo控制版

架构：真机 → /joint_states → 本脚本 → mujoco Python API → MuJoCo仿真窗口

优势：
  - 不需要MuJoCo的ros2_control_node，避免双controller_manager冲突
  - 直接使用mujoco Python包控制仿真，延迟更低
  - 显示独立的MuJoCo GUI窗口
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
import os


class DigitalTwinSyncMujoco(Node):
    """数字孪生同步节点 - 直接MuJoCo控制"""

    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    # MuJoCo模型中对应的actuator名称
    ACTUATOR_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    def __init__(self, model_path):
        super().__init__('digital_twin_sync')

        # 加载MuJoCo模型
        self.get_logger().info(f'Loading MuJoCo model: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 找到actuator索引
        self.actuator_indices = {}
        for name in self.ACTUATOR_NAMES:
            try:
                idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                self.actuator_indices[name] = idx
                self.get_logger().info(f'  Actuator: {name} -> index {idx}')
            except Exception:
                self.get_logger().warn(f'  Actuator {name} not found in model')

        # 参数
        self.declare_parameter('sync_frequency', 50.0)
        self.declare_parameter('lpf_alpha', 0.6)
        self.sync_freq = self.get_parameter('sync_frequency').value
        lpf_alpha = self.get_parameter('lpf_alpha').value

        # 低通滤波
        self.lpf_alpha = lpf_alpha
        self.lpf_last = None

        # 状态
        self.target_positions = None
        self.current_positions = None
        self.state_lock = threading.Lock()
        self.first_state_received = False
        self.sync_count = 0

        # 订阅真机/joint_states
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # MuJoCo仿真线程
        self.viewer_running = True
        self.viewer_thread = threading.Thread(target=self._run_mujoco, daemon=True)
        self.viewer_thread.start()

        self.get_logger().info('Digital Twin Sync (Direct MuJoCo) Started')
        self.get_logger().info(f'  Sync frequency: {self.sync_freq} Hz')
        self.get_logger().info('  Waiting for real robot joint states...')

    def joint_callback(self, msg):
        """接收真机关节状态"""
        positions = []
        for joint_name in self.JOINT_NAMES:
            try:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            except (ValueError, IndexError):
                return

        with self.state_lock:
            # 低通滤波
            if self.lpf_last is not None:
                filtered = [
                    self.lpf_alpha * p + (1 - self.lpf_alpha) * l
                    for p, l in zip(positions, self.lpf_last)
                ]
            else:
                filtered = positions[:]
            self.lpf_last = filtered[:]
            self.target_positions = filtered

        if not self.first_state_received:
            self.get_logger().info('Real robot state received!')
            self.get_logger().info(
                f'  Positions: [{", ".join(f"{p:.3f}" for p in filtered)}]')
            self.first_state_received = True

    def _apply_positions(self):
        """将目标位置应用到MuJoCo模型"""
        with self.state_lock:
            if self.target_positions is None:
                return
            positions = self.target_positions[:]

        # 设置actuator控制信号（位置控制）
        for i, name in enumerate(self.ACTUATOR_NAMES):
            if name in self.actuator_indices:
                idx = self.actuator_indices[name]
                self.data.ctrl[idx] = positions[i]

        self.sync_count += 1

    def _run_mujoco(self):
        """MuJoCo仿真+GUI线程"""
        # 先前进仿真一步初始化
        mujoco.mj_step(self.model, self.data)

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.get_logger().info('MuJoCo viewer opened')

            while viewer.is_running() and self.viewer_running:
                # 应用目标位置
                self._apply_positions()

                # 步进仿真
                mujoco.mj_step(self.model, self.data)

                # 同步viewer
                viewer.sync()

                # 控制仿真速率
                time.sleep(1.0 / self.sync_freq)

        self.get_logger().info('MuJoCo viewer closed')

    def stop(self):
        self.viewer_running = False


def main(args=None):
    rclpy.init(args=args)

    # 查找MuJoCo模型路径
    model_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'src', 'piper_mujoco', 'models', 'piper.xml')

    # 也尝试从install目录查找
    if not os.path.exists(model_path):
        # 尝试通过ament_index_python查找
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('piper_mujoco')
            model_path = os.path.join(pkg_dir, 'models', 'piper.xml')
        except Exception:
            pass

    if not os.path.exists(model_path):
        print(f'ERROR: MuJoCo model not found at {model_path}')
        print('Please ensure piper_mujoco package is built and installed')
        rclpy.shutdown()
        return

    node = DigitalTwinSyncMujoco(model_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
