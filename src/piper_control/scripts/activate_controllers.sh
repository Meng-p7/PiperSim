#!/bin/bash
# Load and activate all controllers sequentially.
CM="${1:-/controller_manager}"

echo "=== Waiting for controller_manager ==="
until ros2 service list 2>/dev/null | grep -q "list_controllers"; do
    sleep 1
done
sleep 3

for CTRL in joint_state_broadcaster piper_arm_controller piper_gripper_controller; do
    echo "=== Processing $CTRL ==="
    ros2 control load_controller "$CTRL" -c "$CM" 2>&1 || true
    sleep 2
done

echo "=== Configuring (inactive) ==="
for CTRL in joint_state_broadcaster piper_arm_controller piper_gripper_controller; do
    ros2 control set_controller_state "$CTRL" inactive -c "$CM" 2>&1 || true
    sleep 1
done

echo "=== Activating ==="
for CTRL in joint_state_broadcaster piper_arm_controller piper_gripper_controller; do
    ros2 control set_controller_state "$CTRL" active -c "$CM" 2>&1 || true
    sleep 1
done

echo "=== Final status ==="
ros2 control list_controllers -c "$CM"
