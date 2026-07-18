// Copyright 2024 Piper Robot
// Piper 六轴机械臂 + 夹爪的 SocketCAN 硬件接口。
//
// 协议：Piper SDK V2（Agilex 官方）
//
// 控制流程：
//   on_activate:
//     1. 发送使能指令（ID 0x471）—— 使能全部 7 个电机
//     2. 发送 MotionCtrl_2（ID 0x151）—— 设置 CAN 控制 + MOVEJ 模式
//
//   read() — 读取所有待处理的 CAN 帧：
//     ID 0x2A5: 关节 1-2 反馈（int32，大端序，单位 0.001°）
//     ID 0x2A6: 关节 3-4 反馈
//     ID 0x2A7: 关节 5-6 反馈
//     ID 0x2A8: 夹爪反馈（int32 角度 0.001mm，int16 力矩，uint8 状态）
//
//   write():
//     ID 0x155: 关节 1-2 指令（int32，大端序，单位 0.001°）
//     ID 0x156: 关节 3-4 指令
//     ID 0x157: 关节 5-6 指令
//     ID 0x159: 夹爪指令（int32 角度 0.001mm，uint16 力矩，uint8 使能码，uint8 保留）

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
  // 步骤 0：先失能以清除错误状态
  cmd_disable_motors();
  usleep(200000);  // 200ms

  // 步骤 1：使能全部电机（ID 0x471）—— 多次发送，参考 SDK 行为
  for (int i = 0; i < 10; ++i)
  {
    cmd_enable_motors();
    usleep(10000);  // 10ms
  }
  usleep(500000);  // 500ms wait for arm to confirm enable
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Motors enabled");

  // 步骤 2：设置运动模式 — CAN 控制 + MOVEJ（ID 0x151）
  if (!cmd_set_motion_mode())
  {
    RCLCPP_ERROR(rclcpp::get_logger("PiperHardware"),
                 "Failed to set motion mode");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "Motion mode set: CAN ctrl + MOVEJ, speed=100%%");

  // 用当前位置初始化指令（先读几帧反馈）
  for (int i = 0; i < 10; ++i)
  {
    drain_can_rx();
    usleep(5000);
  }

  // 步骤 3：移动到零位（参考 SDK 默认行为）
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "Moving to zero position...");
  for (int i = 0; i < 400; ++i)  // ~2 seconds at 5ms per cycle
  {
    cmd_set_motion_mode();
    cmd_write_joint_positions_zero();
    cmd_write_gripper_zero();
    usleep(5000);
  }

  // 读取最终位置作为指令起点
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
  // 安全关闭：先回零再失能
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"),
              "Moving to zero position before disable...");

  // 持续发送零位指令约 3 秒（600 次循环，每次 5ms）
  for (int i = 0; i < 600; ++i)
  {
    cmd_set_motion_mode();
    cmd_write_joint_positions_zero();
    cmd_write_gripper_zero();
    usleep(5000);
  }

  // 安全失能
  cmd_disable_motors();
  std::fill(hw_cmd_.begin(), hw_cmd_.begin() + NUM_CMD_JOINTS, 0.0);
  RCLCPP_INFO(rclcpp::get_logger("PiperHardware"), "At zero, motors disabled");
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
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiperHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 每个周期都设置运动模式（某些固件需要持续发送才能接受指令）
  cmd_set_motion_mode();

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
      // 夹爪：int32 角度（0.001mm）+ int16 力矩 + uint8 状态
      int32_t grip_raw;
      std::memcpy(&grip_raw, d, 4);
      grip_raw = static_cast<int32_t>(ntohl(static_cast<uint32_t>(grip_raw)));
      hw_pos_[6] = static_cast<double>(grip_raw) / METER_TO_UMM;

      // 解析状态字节，检测过热/错误
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
    // 其他 ID（0x2A1 状态、0x2A2-2A4 末端位姿、0x251-256 高速反馈、
    // 0x261-266 低速反馈、0x473/476/478 配置反馈等）静默丢弃。
  }
}

// ---------------------------------------------------------------------------
// Piper SDK V2 协议
// ---------------------------------------------------------------------------

bool PiperHardware::cmd_enable_motors()
{
  // ID 0x471: 电机使能/失能
  // Byte 0: 电机序号（7=全部含夹爪，0xFF=全部关节+夹爪）
  // Byte 1: 使能标志（0x01=失能，0x02=使能）
  uint8_t data[8] = {0x07, 0x02, 0, 0, 0, 0, 0, 0};  // 使能全部 7 个电机
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_disable_motors()
{
  uint8_t data[8] = {0x07, 0x01, 0, 0, 0, 0, 0, 0};  // 失能全部 7 个电机
  return send_can_frame(ID_MOTOR_ENABLE, data, 8);
}

bool PiperHardware::cmd_set_motion_mode()
{
  // ID 0x151: MotionCtrl_2
  // Byte 0: ctrl_mode        0x01 = CAN 指令控制
  // Byte 1: move_mode        0x01 = MOVEJ（关节空间）
  // Byte 2: move_spd_rate    100  = 100% 速度
  // Byte 3: is_mit_mode      0x00 = 位置-速度模式
  // Byte 4: residence_time   0x00
  // Byte 5: installation_pos 0x00 = 默认安装位
  // Byte 6-7: reserved       0x00
  uint8_t data[8] = {0x01, 0x01, 100, 0x00, 0x00, 0x00, 0x00, 0x00};
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
  // ID 0x159: 夹爪指令
  // Byte 0-3: int32 大端序，夹爪角度 0.001mm
  // Byte 4-5: uint16 大端序，力矩 0.001N/m（0-5000）
  // Byte 6:   uint8，使能码（0x01=使能）
  // Byte 7:   uint8，置零（0x00=不操作）
  auto pos_raw = static_cast<int32_t>(
    std::round(std::abs(hw_cmd_[6]) * METER_TO_UMM));

  uint8_t data[8] = {};
  uint32_t be = htonl(static_cast<uint32_t>(pos_raw));
  std::memcpy(data, &be, 4);
  // 力矩 = 1000（1.0 N/m），大端序
  data[4] = 0x03;
  data[5] = 0xE8;  // 1000 = 0x03E8
  data[6] = 0x01;  // 使能夹爪
  data[7] = 0x00;  // 不置零

  return send_can_frame(ID_GRIP_CMD, data, 8);
}

bool PiperHardware::cmd_write_joint_positions_zero()
{
  // 发送全部关节到零位
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
  // 夹爪回零（闭合）
  uint8_t data[8] = {};
  data[4] = 0x03;
  data[5] = 0xE8;  // effort = 1000
  data[6] = 0x01;  // enable
  data[7] = 0x00;
  return send_can_frame(ID_GRIP_CMD, data, 8);
}

}  // namespace piper_control

// 注册为 ros2_control 插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  piper_control::PiperHardware,
  hardware_interface::SystemInterface)
