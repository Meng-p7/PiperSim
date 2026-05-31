// Copyright 2024 Piper Robot
// SocketCAN hardware interface for the Piper 6-DOF arm + gripper.
//
// Protocol: Piper SDK V2 (Agilex official)
//
// Control flow:
//   on_activate:
//     1. Send enable command (ID 0x471) — enable all 7 motors
//     2. Send MotionCtrl_2 (ID 0x151) — set CAN control + MOVEJ mode
//
//   read() — drain all pending CAN frames:
//     ID 0x2A5: joint 1-2 feedback (int32, big-endian, unit 0.001°)
//     ID 0x2A6: joint 3-4 feedback
//     ID 0x2A7: joint 5-6 feedback
//     ID 0x2A8: gripper feedback (int32 angle in 0.001mm, int16 effort, uint8 status)
//
//   write():
//     ID 0x155: joint 1-2 command (int32, big-endian, unit 0.001°)
//     ID 0x156: joint 3-4 command
//     ID 0x157: joint 5-6 command
//     ID 0x159: gripper command (int32 angle in 0.001mm, uint16 effort, uint8 code, uint8 zero)

#include "piper_control/piper_hardware.hpp"

#include <algorithm>
#include <cstring>
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

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn PiperHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_iface_ = "can0";
  if (info.hardware_parameters.count("can_interface"))
  {
    can_iface_ = info.hardware_parameters.at("can_interface");
  }

  hw_pos_.resize(NUM_JOINTS, 0.0);
  hw_vel_.resize(NUM_JOINTS, 0.0);
  hw_cmd_.resize(NUM_JOINTS, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "on_init: can_interface=%s, joints=%zu",
              can_iface_.c_str(), info.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_can(can_iface_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Failed to open CAN interface '%s'", can_iface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "CAN interface '%s' opened (fd=%d)", can_iface_.c_str(), can_fd_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_can();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Step 0: Disable first to clear any error state
  cmd_disable_motors();
  usleep(200000);  // 200ms

  // Step 1: Enable all motors (ID 0x471) — send multiple times like SDK
  for (int i = 0; i < 10; ++i)
  {
    cmd_enable_motors();
    usleep(10000);  // 10ms
  }
  usleep(500000);  // 500ms wait for arm to confirm enable
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Motors enabled");

  // Step 2: Set motion mode — CAN control + MOVEJ (ID 0x151)
  if (!cmd_set_motion_mode())
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Failed to set motion mode");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "Motion mode set: CAN ctrl + MOVEJ, speed=100%%");

  // Initialize command with current positions (read a few frames first)
  for (int i = 0; i < 10; ++i)
  {
    drain_can_rx();
    usleep(5000);
  }

  // Step 3: Move arm to zero position (like SDK default)
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Moving to zero position...");
  for (int i = 0; i < 400; ++i)  // ~2 seconds at 5ms per cycle
  {
    cmd_set_motion_mode();
    cmd_write_joint_positions_zero();
    cmd_write_gripper_zero();
    usleep(5000);
  }

  // Read final positions as command starting point
  for (int i = 0; i < 10; ++i)
  {
    drain_can_rx();
    usleep(5000);
  }
  std::copy(hw_pos_.begin(), hw_pos_.begin() + NUM_CMD_JOINTS, hw_cmd_.begin());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Safety: move to zero position before disabling
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "Moving to zero position before disable...");

  // Send zero position commands for ~3 seconds (600 cycles at 5ms)
  for (int i = 0; i < 600; ++i)
  {
    cmd_set_motion_mode();
    cmd_write_joint_positions_zero();
    cmd_write_gripper_zero();
    usleep(5000);
  }

  // Now safely disable
  cmd_disable_motors();
  std::fill(hw_cmd_.begin(), hw_cmd_.begin() + NUM_CMD_JOINTS, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "At zero, motors disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
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

  // Only joints with command interfaces (joint1-7), joint8 is mimic
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
// Read / Write
// ---------------------------------------------------------------------------

hardware_interface::return_type PiperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  drain_can_rx();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
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
  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// CAN socket helpers
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

  // Set receive timeout
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

bool PiperHardware::recv_can_frame(
  uint32_t expected_id, uint8_t * data, uint8_t * out_len, double timeout_s)
{
  // Wait for data with select() WITHOUT holding the mutex
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    int fd;
    {
      std::lock_guard<std::mutex> lk(can_mtx_);
      fd = can_fd_;
    }
    if (fd < 0) return false;

    FD_SET(fd, &rfds);
    struct timeval tv {};
    tv.tv_sec = static_cast<long>(timeout_s);
    tv.tv_usec = static_cast<long>((timeout_s - tv.tv_sec) * 1e6);

    int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) return false;
  }

  // Read under lock
  std::lock_guard<std::mutex> lk(can_mtx_);
  if (can_fd_ < 0) return false;

  struct can_frame frame {};
  ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
  if (nbytes < static_cast<ssize_t>(sizeof(frame))) return false;
  if (frame.can_id != expected_id) return false;

  std::memcpy(data, frame.data, frame.can_dlc);
  if (out_len) *out_len = frame.can_dlc;
  return true;
}

void PiperHardware::drain_can_rx()
{
  // Read all pending CAN frames and update joint/gripper state.
  // The arm pushes feedback continuously (~200Hz for joints).
  while (true)
  {
    // Non-blocking select
    fd_set rfds;
    FD_ZERO(&rfds);
    int fd;
    {
      std::lock_guard<std::mutex> lk(can_mtx_);
      fd = can_fd_;
    }
    if (fd < 0) return;

    FD_SET(fd, &rfds);
    struct timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 0;  // non-blocking

    int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) return;

    struct can_frame frame {};
    {
      std::lock_guard<std::mutex> lk(can_mtx_);
      if (can_fd_ < 0) return;
      ssize_t n = ::read(can_fd_, &frame, sizeof(frame));
      if (n < static_cast<ssize_t>(sizeof(frame))) return;
    }

    // Decode feedback based on CAN ID (Piper SDK V2 protocol)
    uint32_t id = frame.can_id;
    const uint8_t * d = frame.data;

    if (id == ID_JOINT_FB_12 && frame.can_dlc >= 8)
    {
      // Joint 1-2: two big-endian int32, unit 0.001°
      int32_t j1_raw, j2_raw;
      std::memcpy(&j1_raw, d, 4);
      std::memcpy(&j2_raw, d + 4, 4);
      j1_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j1_raw)));
      j2_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j2_raw)));
      hw_pos_[0] = static_cast<double>(j1_raw) / RAD_TO_MDEG;
      hw_pos_[1] = static_cast<double>(j2_raw) / RAD_TO_MDEG;
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
    }
    else if (id == ID_GRIP_FB && frame.can_dlc >= 7)
    {
      // Gripper: int32 angle (0.001mm) + int16 effort + uint8 status
      int32_t grip_raw;
      std::memcpy(&grip_raw, d, 4);
      grip_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(grip_raw)));
      hw_pos_[6] = static_cast<double>(grip_raw) / METER_TO_UMM;

      // Parse status byte for overheat/error warnings
      uint8_t status = d[6];
      if (status & 0x02) {  // motor_overheating
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Motor overheating detected! Arm may not respond to commands.");
      }
      if (status & 0x08) {  // driver_overheating
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Driver overheating! Arm may not respond to commands.");
      }
      if (status & 0x20) {  // driver_error_status
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("PiperHardware"),
          *rclcpp::Clock::make_shared(), 10000,
          "Driver error detected! Try re-enabling the arm.");
      }
    }
    // All other IDs (0x2A1 status, 0x2A2-2A4 end pose, 0x251-256 high-speed,
    // 0x261-266 low-speed, 0x473/476/478 config feedback, etc.) are silently discarded.
  }
}

// ---------------------------------------------------------------------------
// Piper SDK V2 protocol
// ---------------------------------------------------------------------------

bool PiperHardware::cmd_enable_motors()
{
  // ID 0x471: Motor Enable/Disable
  // Byte 0: motor_num (7 = all motors including gripper, 0xFF = all joints + gripper)
  // Byte 1: enable_flag (0x01 = disable, 0x02 = enable)
  uint8_t data[8] = {0x07, 0x02, 0, 0, 0, 0, 0, 0};  // enable all 7 motors
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_disable_motors()
{
  uint8_t data[8] = {0x07, 0x01, 0, 0, 0, 0, 0, 0};  // disable all 7 motors
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_set_motion_mode()
{
  // ID 0x151: MotionCtrl_2
  // Byte 0: ctrl_mode        0x01 = CAN command control
  // Byte 1: move_mode        0x01 = MOVEJ (joint space)
  // Byte 2: move_spd_rate    100  = 100% speed
  // Byte 3: is_mit_mode      0x00 = position-velocity mode
  // Byte 4: residence_time   0x00
  // Byte 5: installation_pos 0x00 = default
  // Byte 6-7: reserved       0x00
  uint8_t data[8] = {0x01, 0x01, 100, 0x00, 0x00, 0x00, 0x00, 0x00};
  return send_can_frame(ID_MOTION_CTRL_2, data, 8);
}

bool PiperHardware::cmd_write_joint_positions()
{
  // ID 0x155/0x156/0x157: Joint command pairs
  // Each frame: two big-endian int32, unit 0.001°
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

  // Joints 1 & 2 → ID 0x155
  encode_pair(hw_cmd_[0], hw_cmd_[1], data);
  if (!send_can_frame(ID_JOINT_CMD_12, data, 8)) return false;

  // Joints 3 & 4 → ID 0x156
  encode_pair(hw_cmd_[2], hw_cmd_[3], data);
  if (!send_can_frame(ID_JOINT_CMD_34, data, 8)) return false;

  // Joints 5 & 6 → ID 0x157
  encode_pair(hw_cmd_[4], hw_cmd_[5], data);
  if (!send_can_frame(ID_JOINT_CMD_56, data, 8)) return false;

  return true;
}

bool PiperHardware::cmd_write_gripper()
{
  // ID 0x159: Gripper command
  // Byte 0-3: int32 big-endian, gripper angle in 0.001mm
  // Byte 4-5: uint16 big-endian, effort in 0.001N/m (0-5000)
  // Byte 6:   uint8, status code (0x01 = enable)
  // Byte 7:   uint8, set_zero (0x00 = no action)
  auto pos_raw = static_cast<int32_t>(
    std::round(std::abs(hw_cmd_[6]) * METER_TO_UMM));

  uint8_t data[8] = {};
  uint32_t be = htonl(static_cast<uint32_t>(pos_raw));
  std::memcpy(data, &be, 4);
  // Effort = 1000 (1.0 N/m), big-endian
  data[4] = 0x03;
  data[5] = 0xE8;  // 1000 = 0x03E8
  data[6] = 0x01;  // enable gripper
  data[7] = 0x00;  // no zero-set

  return send_can_frame(ID_GRIP_CMD, data, 8);
}

bool PiperHardware::cmd_write_joint_positions_zero()
{
  // Send all joints to zero position
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
  encode_pair(0.0, 0.0, data);
  send_can_frame(ID_JOINT_CMD_12, data, 8);
  encode_pair(0.0, 0.0, data);
  send_can_frame(ID_JOINT_CMD_34, data, 8);
  encode_pair(0.0, 0.0, data);
  send_can_frame(ID_JOINT_CMD_56, data, 8);
  return true;
}

bool PiperHardware::cmd_write_gripper_zero()
{
  // Send gripper to zero (closed)
  uint8_t data[8] = {};
  data[4] = 0x03;
  data[5] = 0xE8;  // effort = 1000
  data[6] = 0x01;  // enable
  data[7] = 0x00;
  return send_can_frame(ID_GRIP_CMD, data, 8);
}

}  // namespace piper_control

// Register as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  piper_control::PiperHardware,
  hardware_interface::SystemInterface)
