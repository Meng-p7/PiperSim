#!/usr/bin/env python3
"""
数字孪生同步脚本 - 直接MuJoCo控制版

架构：真机 → /joint_states → 本脚本 → mujoco Python API → MuJoCo仿真窗口

优势：
  - 不需要MuJoCo的ros2_control_node，避免双controller_manager冲突
  - 直接使用mujoco Python包控制仿真，延迟更低
  - 显示独立的MuJoCo GUI窗口
"""

import os
import sys
import threading
import time
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ModuleNotFoundError:
    print('[FAIL] Python MuJoCo 模块未安装', file=sys.stderr)
    print('       修复: 先按 README 创建 .venv-mujoco，或使用 Docker', file=sys.stderr)
    sys.exit(1)

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class DigitalTwinSyncMujoco(Node):
    """数字孪生同步节点 - 直接MuJoCo控制"""

    JOINT_NAMES = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        'gripper_joint',
    ]
    # MuJoCo模型中对应的actuator名称
    ACTUATOR_NAMES = [
        'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        'gripper',
    ]

    def __init__(self, model_path):
        super().__init__('digital_twin_sync')

        # 加载MuJoCo模型
        self.get_logger().info(f'Loading MuJoCo model: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 找到actuator索引
        self.actuator_indices = {}
        for name in self.ACTUATOR_NAMES:
            idx = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if idx < 0:
                raise ValueError(f'MuJoCo actuator not found: {name}')
            self.actuator_indices[name] = idx
            self.get_logger().info(f'  Actuator: {name} -> index {idx}')

        # 参数
        self.declare_parameter('sync_frequency', 50.0)
        self.declare_parameter('lpf_alpha', 0.6)
        self.sync_freq = float(self.get_parameter('sync_frequency').value)
        lpf_alpha = float(self.get_parameter('lpf_alpha').value)
        if self.sync_freq <= 0.0:
            raise ValueError('sync_frequency must be greater than zero')
        if not 0.0 <= lpf_alpha <= 1.0:
            raise ValueError('lpf_alpha must be in [0, 1]')

        # 低通滤波
        self.lpf_alpha = lpf_alpha
        self.lpf_last = None

        # 状态
        self.target_positions = None
        self.state_lock = threading.Lock()
        self.first_state_received = False
        self.sync_count = 0

        # 订阅真机/joint_states
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)

        # MuJoCo仿真线程
        self.viewer_running = True
        self.viewer_handle = None
        self.viewer_error = None
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
        try:
            # 先前进仿真一步初始化
            mujoco.mj_step(self.model, self.data)

            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                self.viewer_handle = viewer
                self.get_logger().info('MuJoCo viewer opened')

                while viewer.is_running() and self.viewer_running:
                    self._apply_positions()
                    mujoco.mj_step(self.model, self.data)
                    viewer.sync()
                    time.sleep(1.0 / self.sync_freq)

        except Exception as exc:
            self.viewer_error = exc
            self.get_logger().error(f'MuJoCo viewer failed: {exc}')
        finally:
            self.viewer_running = False
            self.viewer_handle = None
            if rclpy.ok():
                self.get_logger().info(
                    'MuJoCo viewer closed; stopping Twin mode')
                rclpy.shutdown()

    def stop(self):
        self.viewer_running = False
        viewer = self.viewer_handle
        if viewer is not None:
            try:
                # MuJoCo documents close() as safe without the viewer lock.
                viewer.close()
            except Exception as exc:
                self.get_logger().warning(
                    f'Failed to close MuJoCo viewer cleanly: {exc}')
        if self.viewer_thread.is_alive():
            self.viewer_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)

    no_graphical_session = (
        not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY')
    )
    if sys.platform.startswith('linux') and no_graphical_session:
        print(
            'ERROR: Twin mode requires a graphical session '
            '(DISPLAY or WAYLAND_DISPLAY).',
            file=sys.stderr,
        )
        print(
            'Use Real mode on a headless host, or enable docker/run.sh --gui.',
            file=sys.stderr,
        )
        rclpy.shutdown()
        return 1

    # 查找MuJoCo模型路径
    model_path = Path(__file__).resolve().parents[1] / 'models' / 'piper.xml'

    # 也尝试从install目录查找
    if not model_path.exists():
        # 尝试通过ament_index_python查找
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('piper_mujoco')
            model_path = Path(pkg_dir) / 'models' / 'piper.xml'
        except Exception:
            pass

    if not model_path.exists():
        print(f'ERROR: MuJoCo model not found at {model_path}')
        print('Please ensure piper_mujoco package is built and installed')
        rclpy.shutdown()
        return 1

    try:
        node = DigitalTwinSyncMujoco(str(model_path))
    except Exception as exc:
        print(f'ERROR: Failed to initialize digital twin: {exc}', file=sys.stderr)
        rclpy.shutdown()
        return 1

    status = 0
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        if node.viewer_error is not None:
            status = 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
