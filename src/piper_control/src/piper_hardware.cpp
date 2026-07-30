// Copyright 2024 Piper Robot
// Piper 六轴机械臂 + 夹爪的 SocketCAN 硬件接口。
//
// 协议：Piper SDK V2（Agilex 官方）
//
// 控制流程：
//   on_activate:
//     1. 等待 6 个关节和夹爪的完整、新鲜反馈
//     2. 使用当前位置初始化指令缓冲区
//     3. 以受限速度进入 CAN 控制并使能电机
//
//   read() — 读取所有待处理的 CAN 帧：
//     ID 0x2A5: 关节 1-2 反馈（int32，大端序，单位 0.001°）
//     ID 0x2A6: 关节 3-4 反馈
//     ID 0x2A7: 关节 5-6 反馈
//     ID 0x2A8: 夹爪反馈（int32 总开口 0.001mm，int16 力矩，uint8 状态）
//
//   write():
//     ID 0x155: 关节 1-2 指令（int32，大端序，单位 0.001°）
//     ID 0x156: 关节 3-4 指令
//     ID 0x157: 关节 5-6 指令
//     ID 0x159: 夹爪指令（int32 两指总开口 0.001mm，uint16 力矩，uint8 使能码，uint8 保留）

#include "piper_control/piper_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace piper_control
{

namespace
{

bool parse_finite_double(const std::string & text, double & value)
{
  try
  {
    size_t consumed = 0;
    value = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(value);
  }
  catch (const std::exception &)
  {
    return false;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn PiperHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto logger = rclcpp::get_logger("PiperHardware");
  const std::array<std::string, NUM_JOINTS> expected_joint_names{{
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint"}};

  if (info.joints.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(
      logger, "Expected %zu joints (joint1-6 + gripper_joint), got %zu",
      NUM_JOINTS, info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    const auto & joint = info.joints[i];
    if (joint.name != expected_joint_names[i])
    {
      RCLCPP_ERROR(
        logger, "Joint %zu must be '%s', got '%s'; interface order is safety-critical",
        i, expected_joint_names[i].c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger, "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (
      joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger, "Joint '%s' must expose exactly one position state interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & command_interface = joint.command_interfaces[0];
    const bool has_min = !command_interface.min.empty();
    const bool has_max = !command_interface.max.empty();
    if (has_min != has_max)
    {
      RCLCPP_ERROR(
        logger, "Joint '%s' must define both command min and max, or neither",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (has_min)
    {
      double min_value = 0.0;
      double max_value = 0.0;
      if (
        !parse_finite_double(command_interface.min, min_value) ||
        !parse_finite_double(command_interface.max, max_value) ||
        min_value >= max_value)
      {
        RCLCPP_ERROR(
          logger, "Joint '%s' has invalid command limits min='%s', max='%s'",
          joint.name.c_str(), command_interface.min.c_str(), command_interface.max.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      command_min_[i] = min_value;
      command_max_[i] = max_value;
    }
  }

  can_iface_ = "can0";
  if (info.hardware_parameters.count("can_interface"))
  {
    can_iface_ = info.hardware_parameters.at("can_interface");
  }
  if (can_iface_.empty())
  {
    RCLCPP_ERROR(logger, "hardware parameter 'can_interface' must not be empty");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.count("calibration_mode"))
  {
    std::string value = info.hardware_parameters.at("calibration_mode");
    std::transform(value.begin(), value.end(), value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (value == "true" || value == "1" || value == "yes")
    {
      calibration_mode_ = true;
    }
    else if (value == "false" || value == "0" || value == "no")
    {
      calibration_mode_ = false;
    }
    else
    {
      RCLCPP_ERROR(
        logger, "Invalid calibration_mode='%s' (expected true/false)",
        value.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto read_numeric_parameter =
    [&info, &logger](
    const char * name, double default_value, double min_value, double max_value,
    double & output) -> bool
    {
      output = default_value;
      const auto parameter = info.hardware_parameters.find(name);
      if (parameter == info.hardware_parameters.end())
      {
        return true;
      }
      if (
        !parse_finite_double(parameter->second, output) ||
        output < min_value || output > max_value)
      {
        RCLCPP_ERROR(
          logger, "Invalid hardware parameter %s='%s' (expected %.6g..%.6g)",
          name, parameter->second.c_str(), min_value, max_value);
        return false;
      }
      return true;
    };

  double speed_percent = static_cast<double>(speed_percent_);
  double feedback_timeout_ms = static_cast<double>(feedback_timeout_ms_);
  if (
    !read_numeric_parameter("speed_percent", 20.0, 1.0, 100.0, speed_percent) ||
    !read_numeric_parameter(
      "feedback_timeout_ms", 250.0, 10.0, 10000.0, feedback_timeout_ms) ||
    !read_numeric_parameter("max_arm_step", 0.02, 1e-6, 1.0, max_arm_step_) ||
    !read_numeric_parameter("max_gripper_step", 0.002, 1e-6, 0.035, max_gripper_step_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (
    std::floor(speed_percent) != speed_percent ||
    std::floor(feedback_timeout_ms) != feedback_timeout_ms)
  {
    RCLCPP_ERROR(
      logger, "speed_percent and feedback_timeout_ms must be whole numbers");
    return hardware_interface::CallbackReturn::ERROR;
  }
  speed_percent_ = static_cast<int>(speed_percent);
  feedback_timeout_ms_ = static_cast<int>(feedback_timeout_ms);

  hw_pos_.resize(NUM_JOINTS, 0.0);
  hw_cmd_.resize(NUM_JOINTS, 0.0);
  last_sent_cmd_.resize(NUM_JOINTS, 0.0);
  reset_feedback_tracking();
  hardware_active_ = false;
  fault_cleanup_failed_ = false;

  RCLCPP_INFO(
    logger,
    "on_init: can=%s, calibration_mode=%s, speed=%d%%, feedback_timeout=%dms, "
    "max_arm_step=%.6frad, max_gripper_step=%.6fm",
    can_iface_.c_str(), calibration_mode_ ? "true" : "false", speed_percent_,
    feedback_timeout_ms_, max_arm_step_, max_gripper_step_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hardware_active_ = false;
  reset_feedback_tracking();
  if (!open_can(can_iface_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Failed to open CAN interface '%s'", can_iface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  fault_cleanup_failed_ = false;

  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "CAN interface '%s' opened (fd=%d)", can_iface_.c_str(), can_fd_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hardware_active_ = false;
  reset_feedback_tracking();
  close_can();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  // A direct shutdown can bypass the usual deactivate transition. Reuse the
  // measured-position hold and explicit arm/gripper disable sequence whenever
  // the normal hardware path still owns an open CAN socket. Calibration mode
  // deliberately remains feedback-only and must not change the teach state.
  if (can_fd_ >= 0)
  {
    if (calibration_mode_)
    {
      hardware_active_ = false;
      RCLCPP_INFO(
        rclcpp::get_logger("PiperHardware"),
        "Calibration mode shutdown: closing CAN without changing motor state");
    }
    else if (hardware_active_)
    {
      result = on_deactivate(previous_state);
    }
  }

  hardware_active_ = false;
  reset_feedback_tracking();
  close_can();
  return result;
}

hardware_interface::CallbackReturn PiperHardware::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto logger = rclcpp::get_logger("PiperHardware");
  hardware_interface::CallbackReturn result =
    hardware_interface::CallbackReturn::SUCCESS;

  // Jazzy can invoke the error transition more than once for one failed
  // read/write cycle. Never turn a failed emergency cleanup into a later
  // SUCCESS merely because the first callback already closed the CAN socket.
  if (fault_cleanup_failed_)
  {
    hardware_active_ = false;
    reset_feedback_tracking();
    close_can();
    RCLCPP_ERROR(
      logger,
      "Previous hardware fault cleanup failed; preserving the unrecoverable error state");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read()/write() errors on an active normal-mode component must leave the
  // robot in the same best-effort safe state as an explicit deactivation:
  // hold only a fresh, validated measured position, then disable arm/gripper.
  // on_deactivate() clears hardware_active_ before sending anything, making a
  // repeated error/shutdown callback idempotent. Calibration mode is strictly
  // feedback-only, so error handling must not alter the teach/motor state.
  if (calibration_mode_)
  {
    hardware_active_ = false;
    RCLCPP_WARN(
      logger,
      "Calibration mode error cleanup: closing CAN without changing motor state");
  }
  else if (hardware_active_ && can_fd_ >= 0)
  {
    RCLCPP_ERROR(
      logger,
      "Hardware lifecycle error: attempting measured-position hold and motor disable");
    result = on_deactivate(previous_state);
  }
  else
  {
    if (hardware_active_)
    {
      RCLCPP_ERROR(
        logger,
        "Hardware lifecycle error while active, but CAN is unavailable; "
        "motor disable could not be sent");
      result = hardware_interface::CallbackReturn::ERROR;
    }
    hardware_active_ = false;
  }

  if (result != hardware_interface::CallbackReturn::SUCCESS)
  {
    fault_cleanup_failed_ = true;
  }
  reset_feedback_tracking();
  close_can();
  return result;
}

hardware_interface::CallbackReturn PiperHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hardware_active_ = false;

  if (!wait_for_fresh_feedback())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"),
      "Activation aborted: did not receive fresh joint1-6 and gripper feedback within %dms",
      feedback_timeout_ms_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_cmd_ = hw_pos_;
  last_sent_cmd_ = hw_pos_;

  // In calibration mode the teach button is handled by the robot controller.
  // ros2_control must only provide feedback; it must not enable/disable motors,
  // switch motion mode, or send the initial zero-valued command buffer.
  if (calibration_mode_)
  {
    hardware_active_ = true;
    RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
                "Calibration mode: CAN feedback only; motion writes disabled");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::string validation_error;
  if (!validate_commands(hw_cmd_, false, validation_error))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"),
      "Activation aborted: current feedback is unsafe: %s", validation_error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Preload the current arm position before enabling. This replaces the old
  // zero-position startup sequence and prevents an activation jump.
  if (!cmd_set_motion_mode() || !cmd_write_joint_positions())
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Activation aborted: failed to preload current arm position");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (int i = 0; i < 3; ++i)
  {
    if (!cmd_enable_motors())
    {
      cmd_disable_motors();
      cmd_disable_gripper();
      RCLCPP_ERROR(
        rclcpp::get_logger("PiperHardware"),
        "Activation aborted: failed to send motor enable command");
      return hardware_interface::CallbackReturn::ERROR;
    }
    usleep(10000);
  }

  // Reinforce the current-position hold immediately after enabling.
  for (int i = 0; i < 3; ++i)
  {
    if (
      !cmd_set_motion_mode() ||
      !cmd_write_joint_positions() ||
      !cmd_write_gripper())
    {
      cmd_disable_motors();
      cmd_disable_gripper();
      RCLCPP_ERROR(
        rclcpp::get_logger("PiperHardware"),
        "Activation aborted: failed to hold the measured position");
      return hardware_interface::CallbackReturn::ERROR;
    }
    usleep(10000);
  }

  hardware_active_ = true;
  RCLCPP_INFO(
    rclcpp::get_logger("PiperHardware"),
    "Motor enable and measured-position hold commands sent at %d%% speed; "
    "this interface does not yet confirm motor state from feedback",
    speed_percent_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hardware_active_ = false;

  if (calibration_mode_)
  {
    // Do not change the robot's teach/hold state when the ROS lifecycle stops.
    RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
                "Calibration mode deactivated; leaving motor state unchanged");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Stop at the measured position. If feedback is stale, do not invent a hold
  // target; disable immediately instead.
  drain_can_rx();
  bool hold_succeeded = true;
  if (feedback_is_fresh(std::chrono::steady_clock::now()))
  {
    hw_cmd_ = hw_pos_;
    last_sent_cmd_ = hw_pos_;
    std::string validation_error;
    if (!validate_commands(hw_cmd_, false, validation_error))
    {
      hold_succeeded = false;
      RCLCPP_WARN(
        rclcpp::get_logger("PiperHardware"),
        "Skipping position hold during deactivation: %s", validation_error.c_str());
    }
    else
    {
      for (int i = 0; i < 3; ++i)
      {
        if (
          !cmd_set_motion_mode() ||
          !cmd_write_joint_positions() ||
          !cmd_write_gripper())
        {
          hold_succeeded = false;
          RCLCPP_WARN(
            rclcpp::get_logger("PiperHardware"),
            "Failed to send position hold during deactivation; disabling immediately");
          break;
        }
        usleep(10000);
      }
    }
  }
  else
  {
    hold_succeeded = false;
    RCLCPP_WARN(
      rclcpp::get_logger("PiperHardware"),
      "Feedback is stale during deactivation; disabling without a position hold");
  }

  bool arm_disable_sent = false;
  bool gripper_disable_sent = false;
  for (int i = 0; i < 3; ++i)
  {
    arm_disable_sent = cmd_disable_motors() || arm_disable_sent;
    gripper_disable_sent = cmd_disable_gripper() || gripper_disable_sent;
    usleep(10000);
  }
  if (!arm_disable_sent || !gripper_disable_sent)
  {
    fault_cleanup_failed_ = true;
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"),
      "Failed to send complete disable commands (arm=%s, gripper=%s)",
      arm_disable_sent ? "sent" : "failed",
      gripper_disable_sent ? "sent" : "failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("PiperHardware"),
    hold_succeeded ?
    "Position-hold and motor-disable commands sent; hardware state is not feedback-confirmed" :
    "Motor-disable commands sent without a position hold; hardware state is not feedback-confirmed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 接口导出
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
PiperHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_pos_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
PiperHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(NUM_CMD_JOINTS);

  // 导出全部 7 个 ros2_control 关节的指令接口（joint1-6 + gripper_joint）；
  // right_finger 是 URDF mimic 关节，不在此列。
  for (size_t i = 0; i < NUM_CMD_JOINTS; ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_cmd_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// 读写周期
// ---------------------------------------------------------------------------

hardware_interface::return_type PiperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  drain_can_rx();
  if (
    hardware_active_ &&
    !feedback_is_fresh(std::chrono::steady_clock::now()))
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("PiperHardware"),
      *rclcpp::Clock::make_shared(), 1000,
      "Hardware read failed: joint/gripper feedback is stale (timeout=%dms)",
      feedback_timeout_ms_);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // controller_manager calls write() even when no trajectory controller is
  // active. Without this guard, the zero-initialized hw_cmd_ is repeatedly sent
  // and is executed as soon as the teach button re-enables the motors.
  if (calibration_mode_)
  {
    return hardware_interface::return_type::OK;
  }
  if (!hardware_active_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!feedback_is_fresh(std::chrono::steady_clock::now()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"),
      "Rejecting command: joint/gripper feedback is stale (timeout=%dms)",
      feedback_timeout_ms_);
    return hardware_interface::return_type::ERROR;
  }

  std::string validation_error;
  if (!validate_commands(hw_cmd_, true, validation_error))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"),
      "Rejecting command: %s", validation_error.c_str());
    return hardware_interface::return_type::ERROR;
  }

  // 每个周期都设置运动模式（某些固件需要持续发送才能接受指令）
  if (!cmd_set_motion_mode())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("PiperHardware"), "CAN motion-mode write failed");
    return hardware_interface::return_type::ERROR;
  }

  if (!cmd_write_joint_positions())
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PiperHardware"),
      *rclcpp::Clock::make_shared(), 5000,
      "CAN joint write failed");
    return hardware_interface::return_type::ERROR;
  }
  if (!cmd_write_gripper())
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PiperHardware"),
      *rclcpp::Clock::make_shared(), 5000,
      "CAN gripper write failed");
    return hardware_interface::return_type::ERROR;
  }
  last_sent_cmd_ = hw_cmd_;
  return hardware_interface::return_type::OK;
}

void PiperHardware::reset_feedback_tracking()
{
  feedback_stamps_.fill(std::chrono::steady_clock::time_point{});
}

bool PiperHardware::wait_for_fresh_feedback()
{
  reset_feedback_tracking();
  const auto timeout = std::chrono::milliseconds(feedback_timeout_ms_);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (std::chrono::steady_clock::now() < deadline)
  {
    drain_can_rx();
    if (feedback_is_fresh(std::chrono::steady_clock::now()))
    {
      return true;
    }
    usleep(5000);
  }

  drain_can_rx();
  return feedback_is_fresh(std::chrono::steady_clock::now());
}

bool PiperHardware::feedback_is_fresh(std::chrono::steady_clock::time_point now) const
{
  const auto timeout = std::chrono::milliseconds(feedback_timeout_ms_);
  const auto empty_stamp = std::chrono::steady_clock::time_point{};
  for (const auto & stamp : feedback_stamps_)
  {
    if (stamp == empty_stamp || now - stamp > timeout)
    {
      return false;
    }
  }
  return true;
}

bool PiperHardware::validate_commands(
  const std::vector<double> & commands, bool check_step, std::string & reason) const
{
  if (commands.size() != NUM_CMD_JOINTS)
  {
    reason = "command vector has an unexpected size";
    return false;
  }
  if (check_step && last_sent_cmd_.size() != NUM_CMD_JOINTS)
  {
    reason = "last-command vector has an unexpected size";
    return false;
  }

  for (size_t i = 0; i < NUM_CMD_JOINTS; ++i)
  {
    const double value = commands[i];
    const std::string & joint_name = info_.joints[i].name;
    if (!std::isfinite(value))
    {
      reason = joint_name + " command is NaN or infinity";
      return false;
    }
    if (value < command_min_[i] || value > command_max_[i])
    {
      std::ostringstream message;
      message << joint_name << " command " << value << " is outside ["
              << command_min_[i] << ", " << command_max_[i] << "]";
      reason = message.str();
      return false;
    }
    if (check_step)
    {
      const double max_step = i < 6 ? max_arm_step_ : max_gripper_step_;
      const double step = std::abs(value - last_sent_cmd_[i]);
      if (!std::isfinite(last_sent_cmd_[i]) || step > max_step)
      {
        std::ostringstream message;
        message << joint_name << " step " << step
                << " exceeds per-cycle limit " << max_step;
        reason = message.str();
        return false;
      }
    }
  }
  reason.clear();
  return true;
}

// ---------------------------------------------------------------------------
// CAN 套接字工具函数
// ---------------------------------------------------------------------------

bool PiperHardware::open_can(const std::string & iface)
{
  std::lock_guard<std::mutex> lk(can_mtx_);

  can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd_ < 0)
  {
    perror("socket(CAN)");
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    perror("ioctl(SIOCGIFINDEX)");
    close(can_fd_);
    can_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    perror("bind(CAN)");
    close(can_fd_);
    can_fd_ = -1;
    return false;
  }

  // 设置接收超时
  struct timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 10000;  // 10 ms
  setsockopt(can_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  return true;
}

void PiperHardware::close_can()
{
  std::lock_guard<std::mutex> lk(can_mtx_);
  if (can_fd_ >= 0)
  {
    close(can_fd_);
    can_fd_ = -1;
  }
}

bool PiperHardware::send_can_frame(uint32_t id, const uint8_t * data, uint8_t len)
{
  std::lock_guard<std::mutex> lk(can_mtx_);
  if (can_fd_ < 0) return false;

  struct can_frame frame {};
  frame.can_id = id;
  frame.can_dlc = len;
  std::memcpy(frame.data, data, len);

  ssize_t nbytes = ::write(can_fd_, &frame, sizeof(frame));
  return nbytes == static_cast<ssize_t>(sizeof(frame));
}

void PiperHardware::drain_can_rx()
{
  // 读取所有待处理的 CAN 帧，更新关节/夹爪状态。
  // 机械臂会持续推送反馈（关节约 200Hz）。
  while (true)
  {
    struct can_frame frame {};
    {
      // 全程持锁：select（非阻塞）与 read 之间不释放锁，
      // 避免其他线程在此窗口内抢先读走帧。
      std::lock_guard<std::mutex> lk(can_mtx_);
      if (can_fd_ < 0) return;

      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(can_fd_, &rfds);
      struct timeval tv {};
      tv.tv_sec = 0;
      tv.tv_usec = 0;  // non-blocking

      int ret = select(can_fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (ret <= 0) return;

      ssize_t n = ::read(can_fd_, &frame, sizeof(frame));
      if (n < static_cast<ssize_t>(sizeof(frame))) return;
    }

    // 根据 CAN ID 解码反馈（Piper SDK V2 协议）
    uint32_t id = frame.can_id;
    const uint8_t * d = frame.data;

    if (id == ID_JOINT_FB_12 && frame.can_dlc >= 8)
    {
      // 关节 1-2：两个大端序 int32，单位 0.001°
      int32_t j1_raw, j2_raw;
      std::memcpy(&j1_raw, d, 4);
      std::memcpy(&j2_raw, d + 4, 4);
      j1_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j1_raw)));
      j2_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j2_raw)));
      hw_pos_[0] = static_cast<double>(j1_raw) / RAD_TO_MDEG;
      hw_pos_[1] = static_cast<double>(j2_raw) / RAD_TO_MDEG;
      feedback_stamps_[0] = std::chrono::steady_clock::now();
    }
    else if (id == ID_JOINT_FB_34 && frame.can_dlc >= 8)
    {
      int32_t j3_raw, j4_raw;
      std::memcpy(&j3_raw, d, 4);
      std::memcpy(&j4_raw, d + 4, 4);
      j3_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j3_raw)));
      j4_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j4_raw)));
      hw_pos_[2] = static_cast<double>(j3_raw) / RAD_TO_MDEG;
      hw_pos_[3] = static_cast<double>(j4_raw) / RAD_TO_MDEG;
      feedback_stamps_[1] = std::chrono::steady_clock::now();
    }
    else if (id == ID_JOINT_FB_56 && frame.can_dlc >= 8)
    {
      int32_t j5_raw, j6_raw;
      std::memcpy(&j5_raw, d, 4);
      std::memcpy(&j6_raw, d + 4, 4);
      j5_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j5_raw)));
      j6_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j6_raw)));
      hw_pos_[4] = static_cast<double>(j5_raw) / RAD_TO_MDEG;
      hw_pos_[5] = static_cast<double>(j6_raw) / RAD_TO_MDEG;
      feedback_stamps_[2] = std::chrono::steady_clock::now();
    }
    else if (id == ID_GRIP_FB && frame.can_dlc >= 7)
    {
      // Piper SDK V2 defines bytes 0-6; byte 7 is reserved and deliberately
      // ignored for compatibility with firmware that sends either DLC 7 or 8.
      // The CAN value is total jaw opening (0..70 mm), while the ROS joint is
      // one finger's travel (0..35 mm); the mimic finger supplies the other half.
      // 夹爪：int32 总开口（0.001mm）+ int16 力矩 + uint8 状态
      int32_t grip_raw;
      std::memcpy(&grip_raw, d, 4);
      grip_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(grip_raw)));
      hw_pos_[6] =
        static_cast<double>(grip_raw) /
        (METER_TO_UMM * GRIPPER_OPENING_PER_JOINT);
      feedback_stamps_[3] = std::chrono::steady_clock::now();

      // Decode documented gripper fault bits. Bit 6 is enable status and bit 7
      // is the homing status; those are states rather than faults.
      uint8_t status = d[6];
      if (status & 0x01)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper undervoltage detected");
      }
      if (status & 0x02)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper motor overheating detected");
      }
      if (status & 0x04)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper driver overcurrent detected");
      }
      if (status & 0x08)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper driver overheating detected");
      }
      if (status & 0x10)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper sensor fault detected");
      }
      if (status & 0x20)
      {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Gripper driver fault detected; inspect hardware before clearing it");
      }
    }
    // 其他 ID（0x2A1 状态、0x2A2-2A4 末端位姿、0x251-256 高速反馈、
    // 0x261-266 低速反馈、0x473/476/478 配置反馈等）静默丢弃。
  }
}

// ---------------------------------------------------------------------------
// Piper SDK V2 协议
// ---------------------------------------------------------------------------

bool PiperHardware::cmd_enable_motors()
{
  // ID 0x471: six arm motors enable/disable. The gripper has an independent
  // enable byte in ID 0x159 and is handled by cmd_write/disable_gripper().
  // Byte 0: motor number (7 is the protocol's all-arm-motors selector)
  // Byte 1: 使能标志（0x01=失能，0x02=使能）
  uint8_t data[8] = {0x07, 0x02, 0, 0, 0, 0, 0, 0};
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_disable_motors()
{
  uint8_t data[8] = {0x07, 0x01, 0, 0, 0, 0, 0, 0};
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_set_motion_mode()
{
  // ID 0x151: MotionCtrl_2
  // Byte 0: ctrl_mode        0x01 = CAN 指令控制
  // Byte 1: move_mode        0x01 = MOVEJ（关节空间）
  // Byte 2: move_spd_rate    1..100 = speed_percent_
  // Byte 3: is_mit_mode      0x00 = 位置-速度模式
  // Byte 4: residence_time   0x00
  // Byte 5: installation_pos 0x00 = invalid/no-op (do not change installation)
  // Byte 6-7: reserved       0x00
  uint8_t data[8] = {
    0x01, 0x01, static_cast<uint8_t>(speed_percent_), 0x00, 0x00, 0x00, 0x00, 0x00};
  return send_can_frame(ID_MOTION_CTRL_2, data, 8);
}

bool PiperHardware::cmd_write_joint_positions()
{
  // ID 0x155/0x156/0x157: 关节指令对
  // 每帧：两个大端序 int32，单位 0.001°
  auto encode_pair = [](double v1, double v2, uint8_t * out)
  {
    auto c1 = static_cast<int32_t>(std::round(v1 * RAD_TO_MDEG));
    auto c2 = static_cast<int32_t>(std::round(v2 * RAD_TO_MDEG));
    uint32_t be1 = htonl(static_cast<uint32_t>(c1));
    uint32_t be2 = htonl(static_cast<uint32_t>(c2));
    std::memcpy(out, &be1, 4);
    std::memcpy(out + 4, &be2, 4);
  };

  uint8_t data[8];

  // 关节 1 & 2 → ID 0x155
  encode_pair(hw_cmd_[0], hw_cmd_[1], data);
  if (!send_can_frame(ID_JOINT_CMD_12, data, 8)) return false;

  // 关节 3 & 4 → ID 0x156
  encode_pair(hw_cmd_[2], hw_cmd_[3], data);
  if (!send_can_frame(ID_JOINT_CMD_34, data, 8)) return false;

  // 关节 5 & 6 → ID 0x157
  encode_pair(hw_cmd_[4], hw_cmd_[5], data);
  if (!send_can_frame(ID_JOINT_CMD_56, data, 8)) return false;

  return true;
}

bool PiperHardware::cmd_write_gripper()
{
  return cmd_send_gripper_control(0x01);
}

bool PiperHardware::cmd_disable_gripper()
{
  return cmd_send_gripper_control(0x00);
}

bool PiperHardware::cmd_send_gripper_control(uint8_t status_code)
{
  // ID 0x159: 夹爪指令
  // Byte 0-3: int32 大端序，两指总开口 0.001mm
  // Byte 4-5: uint16 大端序，力矩 0.001N/m（0-5000）
  // Byte 6:   uint8，状态码（0x00=失能，0x01=使能）
  // Byte 7:   uint8，置零（0x00=不操作）
  auto pos_raw = static_cast<int32_t>(
    std::round(
      hw_cmd_[6] * GRIPPER_OPENING_PER_JOINT * METER_TO_UMM));

  uint8_t data[8] = {};
  uint32_t be = htonl(static_cast<uint32_t>(pos_raw));
  std::memcpy(data, &be, 4);
  // 力矩 = 1000（1.0 N/m），大端序
  data[4] = 0x03;
  data[5] = 0xE8;  // 1000 = 0x03E8
  data[6] = status_code;
  data[7] = 0x00;  // 不置零

  return send_can_frame(ID_GRIP_CMD, data, 8);
}

}  // namespace piper_control

// 注册为 ros2_control 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  piper_control::PiperHardware,
  hardware_interface::SystemInterface)
