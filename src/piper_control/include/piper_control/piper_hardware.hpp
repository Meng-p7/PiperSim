// Copyright 2024 Piper Robot
// CAN bus hardware interface for ros2_control.
// Uses SocketCAN (linux/can.h) to communicate with the Piper arm controller.
// Protocol matches piper_sdk V2 (Agilex official SDK).

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
                      uint8_t * out_len, double timeout_s = 0.001);
  void drain_can_rx();

  // Piper SDK V2 protocol commands
  bool cmd_enable_motors();
  bool cmd_disable_motors();
  bool cmd_set_motion_mode();
  bool cmd_write_joint_positions();
  bool cmd_write_gripper();

  // Data
  int can_fd_ = -1;
  std::string can_iface_;
  mutable std::mutex can_mtx_;

  // 7 DOF: 6 arm joints + 1 gripper
  static constexpr size_t NUM_JOINTS = 7;
  std::vector<double> hw_pos_;
  std::vector<double> hw_vel_;
  std::vector<double> hw_cmd_;

  // Conversion factors (matching piper_sdk V2)
  static constexpr double RAD_TO_MDEG = 57295.7795;   // 1000 * 180 / pi
  static constexpr double METER_TO_UMM = 1000000.0;   // m -> 0.001mm

  // CAN IDs — Piper SDK V2 protocol
  // Motion control
  static constexpr uint32_t ID_MOTION_CTRL_2 = 0x151;  // Mode/move control

  // Joint command
  static constexpr uint32_t ID_JOINT_CMD_12 = 0x155;   // Joint 1-2 command
  static constexpr uint32_t ID_JOINT_CMD_34 = 0x156;   // Joint 3-4 command
  static constexpr uint32_t ID_JOINT_CMD_56 = 0x157;   // Joint 5-6 command

  // Gripper command
  static constexpr uint32_t ID_GRIP_CMD = 0x159;       // Gripper command

  // Motor enable/disable
  static constexpr uint32_t ID_MOTOR_ENABLE = 0x471;   // Motor enable/disable

  // Joint state feedback (pushed by arm, ~200Hz)
  static constexpr uint32_t ID_JOINT_FB_12 = 0x2A5;   // Joint 1-2 feedback
  static constexpr uint32_t ID_JOINT_FB_34 = 0x2A6;   // Joint 3-4 feedback
  static constexpr uint32_t ID_JOINT_FB_56 = 0x2A7;   // Joint 5-6 feedback

  // Gripper state feedback (pushed by arm)
  static constexpr uint32_t ID_GRIP_FB = 0x2A8;       // Gripper feedback
};

}  // namespace piper_control

#endif  // PIPER_CONTROL__PIPER_HARDWARE_HPP_
