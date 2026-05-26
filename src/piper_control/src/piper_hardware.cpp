// Copyright 2024 Piper Robot
// SocketCAN hardware interface for the Piper 6-DOF arm + gripper.
//
// Protocol summary (extend/adjust IDs and frame layout to match your firmware):
//   MotionCtrl : ID=0x010, 8 bytes  – sets control mode (position, speed=100)
//   JointCmd   : ID=0x101..0x103, each frame carries 2 x int32_t (big-endian, 0.001-deg)
//   GripCmd    : ID=0x105, 8 bytes  – position(um), speed, mode, ack
//   JointState : ID=0x201..0x203, same layout as JointCmd
//   GripState  : ID=0x205, 8 bytes

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

  // Read CAN interface name from <param> inside <hardware>
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
  if (!cmd_enable())
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Failed to enable Piper arm");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Piper arm enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiperHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  cmd_disable();
  std::fill(hw_cmd_.begin(), hw_cmd_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Piper arm disabled");
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
  interfaces.reserve(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_cmd_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// Read / Write  (called every control cycle by the controller manager)
// ---------------------------------------------------------------------------

hardware_interface::return_type PiperHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!cmd_read_joint_positions())
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PiperHardware"),
      *rclcpp::Clock::make_shared(), 5000,
      "CAN read failed – keeping last known positions");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!cmd_write_joint_positions() || !cmd_write_gripper())
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("PiperHardware"),
      *rclcpp::Clock::make_shared(), 5000,
      "CAN write failed");
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

  // Set receive timeout so recv() does not block indefinitely
  struct timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 10000;  // 10 ms default
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
  // Use select() to wait for data WITHOUT holding the mutex,
  // so concurrent send_can_frame() calls are not blocked.
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
    if (ret <= 0) return false;  // timeout or error
  }

  // Data is ready – read under lock (fast, non-blocking now)
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

// ---------------------------------------------------------------------------
// Piper CAN protocol
// ---------------------------------------------------------------------------

bool PiperHardware::cmd_enable()
{
  if (!cmd_set_mode()) return false;

  // Send enable command: ID=0x010, byte0=0x01 (enable)
  uint8_t data[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
  if (!send_can_frame(ID_MOTION_CTRL, data, 8)) return false;

  // Wait for ACK
  uint8_t rx[8];
  uint8_t rx_len = 0;
  return recv_can_frame(ID_MOTION_CTRL, rx, &rx_len, 2.0);
}

bool PiperHardware::cmd_disable()
{
  uint8_t data[8] = {0x00, 0, 0, 0, 0, 0, 0, 0};  // disable
  return send_can_frame(ID_MOTION_CTRL, data, 8);
}

bool PiperHardware::cmd_set_mode()
{
  // Set position control mode, speed=100, no ACK request
  // Layout: [mode(1), sub_mode(1), speed(1), ack(1), 0, 0, 0, 0]
  uint8_t data[8] = {0x01, 0x01, 100, 0x00, 0, 0, 0, 0};
  return send_can_frame(ID_MOTION_CTRL, data, 8);
}

bool PiperHardware::cmd_read_joint_positions()
{
  // Request joint state (poll mode)
  uint8_t req[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
  if (!send_can_frame(0x000, req, 1)) return false;

  // Receive 3 state frames: joints 1-2, 3-4, 5-6
  // Use tight 10ms timeouts so the control loop stays within 20ms budget
  constexpr double RECV_TIMEOUT = 0.01;
  uint8_t buf[8];
  uint8_t len = 0;

  // Frame 1: joints 1 & 2
  if (!recv_can_frame(ID_JOINT_STATE_1, buf, &len, RECV_TIMEOUT)) return false;
  {
    int32_t j1_raw, j2_raw;
    std::memcpy(&j1_raw, buf, 4);
    std::memcpy(&j2_raw, buf + 4, 4);
    // Big-endian -> host byte order
    j1_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j1_raw)));
    j2_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j2_raw)));
    hw_pos_[0] = static_cast<double>(j1_raw) / CAN_FACTOR;
    hw_pos_[1] = static_cast<double>(j2_raw) / CAN_FACTOR;
  }

  // Frame 2: joints 3 & 4
  if (!recv_can_frame(ID_JOINT_STATE_2, buf, &len, RECV_TIMEOUT)) return false;
  {
    int32_t j3_raw, j4_raw;
    std::memcpy(&j3_raw, buf, 4);
    std::memcpy(&j4_raw, buf + 4, 4);
    j3_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j3_raw)));
    j4_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j4_raw)));
    hw_pos_[2] = static_cast<double>(j3_raw) / CAN_FACTOR;
    hw_pos_[3] = static_cast<double>(j4_raw) / CAN_FACTOR;
  }

  // Frame 3: joints 5 & 6
  if (!recv_can_frame(ID_JOINT_STATE_3, buf, &len, RECV_TIMEOUT)) return false;
  {
    int32_t j5_raw, j6_raw;
    std::memcpy(&j5_raw, buf, 4);
    std::memcpy(&j6_raw, buf + 4, 4);
    j5_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j5_raw)));
    j6_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(j6_raw)));
    hw_pos_[4] = static_cast<double>(j5_raw) / CAN_FACTOR;
    hw_pos_[5] = static_cast<double>(j6_raw) / CAN_FACTOR;
  }

  // Frame 4: gripper state
  if (!recv_can_frame(ID_GRIP_STATE, buf, &len, RECV_TIMEOUT)) return false;
  {
    uint32_t grip_raw;
    std::memcpy(&grip_raw, buf, 4);
    grip_raw = ntohl(grip_raw);
    hw_pos_[6] = static_cast<double>(grip_raw) / GRIP_FACTOR;
  }

  return true;
}

bool PiperHardware::cmd_write_joint_positions()
{
  auto encode_pair = [](double v1, double v2, uint8_t * out)
  {
    auto c1 = static_cast<int32_t>(std::round(v1 * CAN_FACTOR));
    auto c2 = static_cast<int32_t>(std::round(v2 * CAN_FACTOR));
    uint32_t be1 = htonl(static_cast<uint32_t>(c1));
    uint32_t be2 = htonl(static_cast<uint32_t>(c2));
    std::memcpy(out, &be1, 4);
    std::memcpy(out + 4, &be2, 4);
  };

  uint8_t data[8];

  // Frame 1: joints 1 & 2
  encode_pair(hw_cmd_[0], hw_cmd_[1], data);
  if (!send_can_frame(ID_JOINT_CMD_1, data, 8)) return false;

  // Frame 2: joints 3 & 4
  encode_pair(hw_cmd_[2], hw_cmd_[3], data);
  if (!send_can_frame(ID_JOINT_CMD_2, data, 8)) return false;

  // Frame 3: joints 5 & 6
  encode_pair(hw_cmd_[4], hw_cmd_[5], data);
  if (!send_can_frame(ID_JOINT_CMD_3, data, 8)) return false;

  return true;
}

bool PiperHardware::cmd_write_gripper()
{
  auto pos_raw = static_cast<uint32_t>(
    std::round(std::abs(hw_cmd_[6]) * GRIP_FACTOR));

  uint8_t data[8] = {};
  uint32_t be = htonl(pos_raw);
  std::memcpy(data, &be, 4);
  // bytes 4-5: speed (1000 = 100%), bytes 6-7: mode + ack
  data[4] = 0x03;
  data[5] = 0xE8;  // 1000 big-endian low byte
  data[6] = 0x01;  // mode

  return send_can_frame(ID_GRIP_CMD, data, 8);
}

void PiperHardware::log_joint_state()
{
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] grip: %.4f",
              hw_pos_[0], hw_pos_[1], hw_pos_[2],
              hw_pos_[3], hw_pos_[4], hw_pos_[5],
              hw_pos_[6]);
}

}  // namespace piper_control

// Register as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  piper_control::PiperHardware,
  hardware_interface::SystemInterface)
