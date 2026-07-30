// Copyright 2024 Piper Robot
// ros2_control 的 CAN 总线硬件接口。
// 通过 SocketCAN (linux/can.h) 与 Piper 机械臂控制器通信。
// 协议匹配 piper_sdk V2（Agilex 官方 SDK）。

#ifndef PIPER_CONTROL__PIPER_HARDWARE_HPP_
#define PIPER_CONTROL__PIPER_HARDWARE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
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

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
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
  void drain_can_rx();
  void reset_feedback_tracking();
  bool wait_for_fresh_feedback();
  bool feedback_is_fresh(std::chrono::steady_clock::time_point now) const;
  bool validate_commands(
    const std::vector<double> & commands, bool check_step, std::string & reason) const;

  // Piper SDK V2 protocol commands
  bool cmd_enable_motors();
  bool cmd_disable_motors();
  bool cmd_set_motion_mode();
  bool cmd_write_joint_positions();
  bool cmd_write_gripper();
  bool cmd_disable_gripper();
  bool cmd_send_gripper_control(uint8_t status_code);

  // Data
  int can_fd_ = -1;
  std::string can_iface_;
  bool calibration_mode_ = false;
  bool hardware_active_ = false;
  bool fault_cleanup_failed_ = false;
  mutable std::mutex can_mtx_;

  // 7 DOF: 6 arm + 1 gripper (right_finger 是 URDF mimic 关节，不参与指令/反馈)
  static constexpr size_t NUM_JOINTS = 7;
  static constexpr size_t NUM_CMD_JOINTS = 7;  // joint1-6 + gripper_joint 均有指令接口
  static constexpr size_t NUM_FEEDBACK_GROUPS = 4;  // joint12, joint34, joint56, gripper
  std::vector<double> hw_pos_;
  std::vector<double> hw_cmd_;
  std::vector<double> last_sent_cmd_;
  std::array<double, NUM_JOINTS> command_min_{{
    -2.6179, 0.0, -2.697, -1.745, -1.22, -2.09439, 0.0}};
  std::array<double, NUM_JOINTS> command_max_{{
    2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439, 0.035}};
  std::array<std::chrono::steady_clock::time_point, NUM_FEEDBACK_GROUPS> feedback_stamps_{};

  // Safety parameters, optionally overridden by <hardware><param ...> in the URDF.
  int speed_percent_ = 20;
  int feedback_timeout_ms_ = 250;
  double max_arm_step_ = 0.02;       // radians per write cycle
  double max_gripper_step_ = 0.002;  // metres per write cycle

  // Conversion factors (matching piper_sdk V2)
  static constexpr double RAD_TO_MDEG = 57295.7795;   // 1000 * 180 / pi
  static constexpr double METER_TO_UMM = 1000000.0;   // m -> 0.001mm
  static constexpr double GRIPPER_OPENING_PER_JOINT = 2.0;

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
