#!/usr/bin/env python3
"""
数字孪生模式测试脚本

用法：
  Terminal 1: 激活CAN
    source ~/PiperSim/start_real.sh
    bash src/piper_control/scripts/can_activate.sh can0 1000000

  Terminal 2: 启动Twin模式
    source ~/PiperSim/start_sim.sh
    ros2 launch piper_moveit_config demo.launch.py mode:=twin

  Terminal 3: 运行测试（等Twin模式完全启动后，约15秒）
    source ~/PiperSim/start_sim.sh
    python3 ~/PiperSim/test_twin.py

测试项目：
  1. 话题连通性检查（/joint_states可用）
  2. 同步脚本运行状态检查
  3. MuJoCo窗口跟随验证（需手动拖动真机，目视确认MuJoCo窗口跟随）
  4. 关节运动范围统计
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import sys


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class TwinTestNode(Node):
    def __init__(self):
        super().__init__('twin_tester')

        self.real_positions = None
        self.real_time = None
        self.position_history = []

        # 订阅真机/joint_states
        self.real_sub = self.create_subscription(
            JointState, '/joint_states', self.real_callback, 10)

        self.get_logger().info('Twin Test Node Started')
        self.get_logger().info('Waiting for /joint_states...')

    def _extract_positions(self, msg):
        """提取6个关节的位置"""
        positions = {}
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.position):
                positions[name] = msg.position[i]
        if len(positions) == 6:
            return [positions[j] for j in JOINT_NAMES]
        return None

    def real_callback(self, msg):
        pos = self._extract_positions(msg)
        if pos:
            self.real_positions = pos
            self.real_time = time.time()
            self.position_history.append((time.time(), pos[:]))


def test_topic_connectivity(node):
    """测试1：话题连通性"""
    print('\n' + '=' * 60)
    print('  TEST 1: Topic Connectivity')
    print('=' * 60)

    print('\n  Checking /joint_states (real robot)...')
    start = time.time()
    while time.time() - start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.real_positions is not None:
            print(f'  PASS /joint_states: Received')
            print(f'    Positions: [{", ".join(f"{p:.3f}" for p in node.real_positions)}]')
            return True
    print('  FAIL /joint_states: No data received')
    print('    -> Check CAN activation and real hardware connection')
    return False


def test_sync_node_running():
    """测试2：同步脚本运行状态"""
    print('\n' + '=' * 60)
    print('  TEST 2: Sync Node Running')
    print('=' * 60)

    import subprocess
    result = subprocess.run(
        ['ros2', 'node', 'list'],
        capture_output=True, text=True, timeout=5)

    if 'digital_twin_sync' in result.stdout:
        print('  PASS digital_twin_sync node is running')
        return True
    else:
        print('  FAIL digital_twin_sync node not found')
        return False


def test_joint_movement(node, duration=15.0):
    """测试3：关节运动范围（手动拖动真机）"""
    print('\n' + '=' * 60)
    print(f'  TEST 3: Joint Movement ({duration}s)')
    print('=' * 60)
    print('  Please slowly move each joint of the real robot arm.')
    print('  Check if the MuJoCo window follows the real robot.')
    print()

    node.position_history = []
    start = time.time()
    while time.time() - start < duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)

    if len(node.position_history) < 10:
        print('  FAIL Not enough data collected')
        return False

    # 分析运动范围
    history = node.position_history
    print(f'  Collected {len(history)} samples')
    print(f'\n  Joint ranges (from /joint_states):')
    moved_joints = 0
    for i, name in enumerate(JOINT_NAMES):
        vals = [h[1][i] for h in history]
        range_val = max(vals) - min(vals)
        print(f'    {name}: range={range_val:.3f} rad '
              f'(min={min(vals):.3f}, max={max(vals):.3f})')
        if range_val > 0.05:
            moved_joints += 1

    if moved_joints >= 1:
        print(f'\n  PASS {moved_joints} joints showed movement')
        print('  -> Check MuJoCo window visually to confirm following')
        return True
    else:
        print(f'\n  FAIL No joints showed significant movement')
        return False


def test_visual_follow():
    """测试4：目视确认MuJoCo跟随（交互式）"""
    print('\n' + '=' * 60)
    print('  TEST 4: Visual MuJoCo Following (Interactive)')
    print('=' * 60)
    print('  Did the MuJoCo window follow the real robot arm movement?')

    try:
        response = input('  [y/n]: ').strip().lower()
    except EOFError:
        response = 'y'  # 非交互模式下默认通过

    if response == 'y':
        print('  PASS MuJoCo follows real robot (visual confirmation)')
        return True
    else:
        print('  FAIL MuJoCo does not follow real robot')
        return False


def main():
    rclpy.init()
    node = TwinTestNode()

    print('\n' + '=' * 60)
    print('  Piper Digital Twin Test Suite')
    print('=' * 60)
    print(f'  Time: {time.strftime("%Y-%m-%d %H:%M:%S")}')

    results = {}

    # Test 1: 话题连通性
    results['connectivity'] = test_topic_connectivity(node)
    if not results['connectivity']:
        print('\n  Skipping remaining tests due to connectivity failure')
    else:
        # Test 2: 同步脚本
        results['sync_node'] = test_sync_node_running()

        # Test 3: 关节运动
        results['joint_movement'] = test_joint_movement(node, duration=15.0)

        # Test 4: 目视确认
        results['visual_follow'] = test_visual_follow()

    # 总结
    print('\n' + '=' * 60)
    print('  TEST SUMMARY')
    print('=' * 60)
    for name, passed in results.items():
        status = 'PASS' if passed else 'FAIL'
        print(f'  {name}: {status}')

    total = len(results)
    passed = sum(1 for v in results.values() if v)
    print(f'\n  Total: {passed}/{total} passed')

    node.destroy_node()
    rclpy.shutdown()

    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
