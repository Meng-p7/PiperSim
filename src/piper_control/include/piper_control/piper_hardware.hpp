// Copyright 2024 Piper Robot
// CAN bus hardware interface for ros2_control.
// Uses SocketCAN (linux/can.h) to communicate with the Piper arm controller.

#ifndef PIPER_CONTROL__PIPER_HARDWARE_HPP_
#define PIPER_CONTROL__PIPER_HARDWARE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace piper_control
{

class PiperHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PiperHardware)

  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read / Write cycle
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN helpers
  bool open_can(const std::string & iface);
  void close_can();
  bool send_can_frame(uint32_t id, const uint8_t * data, uint8_t len);
  bool recv_can_frame(uint32_t expected_id, uint8_t * data,
                      uint8_t * out_len, double timeout_s = 0.1);

  // Protocol commands
  bool cmd_enable();
  bool cmd_disable();
  bool cmd_set_mode();
  bool cmd_read_joint_positions();
  bool cmd_write_joint_positions();
  bool cmd_write_gripper();

  void log_joint_state();

  // Socket
  int can_fd_ = -1;
  std::string can_iface_;
  mutable std::mutex can_mtx_;

  // Joint data (7 DOF: 6 arm + 1 gripper)
  static constexpr size_t NUM_ARM_JOINTS = 6;
  static constexpr size_t NUM_JOINTS = 7;
  std::vector<double> hw_pos_;
  std::vector<double> hw_vel_;
  std::vector<double> hw_cmd_;

  // CAN protocol constants
  static constexpr double CAN_FACTOR = 57295.7795;   // rad -> 0.001 deg
  static constexpr double GRIP_FACTOR = 1000000.0;   // m -> um

  // CAN arbitration IDs (Piper protocol)
  static constexpr uint32_t ID_MOTION_CTRL  = 0x010;   // Control mode
  static constexpr uint32_t ID_JOINT_CMD_1  = 0x101;   // Joints 1-2 command
  static constexpr uint32_t ID_JOINT_CMD_2  = 0x102;   // Joints 3-4 command
  static constexpr uint32_t ID_JOINT_CMD_3  = 0x103;   // Joints 5-6 command
  static constexpr uint32_t ID_GRIP_CMD     = 0x105;   // Gripper command
  static constexpr uint32_t ID_JOINT_STATE_1 = 0x201;  // Joints 1-2 state
  static constexpr uint32_t ID_JOINT_STATE_2 = 0x202;  // Joints 3-4 state
  static constexpr uint32_t ID_JOINT_STATE_3 = 0x203;  // Joints 5-6 state
  static constexpr uint32_t ID_GRIP_STATE    = 0x205;   // Gripper state
};

}  // namespace piper_control

#endif  // PIPER_CONTROL__PIPER_HARDWARE_HPP_
