#ifndef ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_
#define ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <memory>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/logging.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>
#include <type_traits>
namespace odrive_can_driver
{

template <typename T>
auto FloatToIntegerBuffer(T value)
{
  static_assert(std::is_floating_point_v<T>, "T must be a floating point type");
  // We use memcpy, because bit_cast is unavailable in C++17 and reinterpret_cast is UB in this case
  if (sizeof(T) == 2) {
    uint16_t result = 0;
    std::memcpy(&result, &value, sizeof(T));
    return result;
  } else if (sizeof(T) == 4) {
    uint32_t result = 0;
    std::memcpy(&result, &value, sizeof(T));
    return result;
    //NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  } else if (sizeof(T) == 8) {
    uint64_t result = 0;
    std::memcpy(&result, &value, sizeof(T));
    return result;
  } else {
    static_assert(!std::is_same_v<T, void>, "Unsupported floating point size");
  }
}

template <typename... T>
std::array<std::byte, (sizeof(T) + ...)> PackToLittleEndian(T &&... args)
{
  std::array<std::byte, (sizeof(T) + ...)> result{};
  size_t result_index = 0;
  auto to_unsigned_buffer = [](auto && arg) {
    if (std::is_floating_point_v<decltype(arg)>) {
      return FloatToIntegerBuffer(arg);
    } else {
      return static_cast<std::make_unsigned_t<std::decay_t<decltype(arg)>>>(arg);
    }
  };
  auto pack_to_result = [&result, &result_index, to_unsigned_buffer](auto && arg) {
    static_assert(
      std::is_floating_point_v<decltype(arg)> || std::is_integral_v<decltype(arg)>,
      "Only floating point and integral types are supported");
    auto unsigned_arg = to_unsigned_buffer(arg);
    const uint8_t k_byte_mask = 0xFF;
    const uint8_t k_bits_in_byte = 8;
    for (size_t arg_index = 0; arg_index < sizeof(unsigned_arg); ++arg_index) {
      result.at(result_index) =
        std::byte((unsigned_arg >> arg_index * k_bits_in_byte) & k_byte_mask);
      ++result_index;
    }
  };

  (pack_to_result(std::forward<T>(args)), ...);

  return result;
}
class CanReadThread
{
public:
  CanReadThread(
    const std::string & can_interface, std::array<MotorAxis, 2> & motor_axis,
    const uint8_t number_of_joints)
  : motor_axis_(motor_axis), number_of_joints_(number_of_joints)
  {
    receiver_ = drivers::socketcan::SocketCanReceiver(can_interface);
    sender_ = drivers::socketcan::SocketCanSender(can_interface);
    thread_ = std::thread([this]() { this->operator()(); });
  }
  void operator()() { (void)number_of_joints_; };

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  uint8_t number_of_joints_{};
  std::thread thread_;
  drivers::socketcan::SocketCanReceiver receiver_;
  drivers::socketcan::SocketCanSender sender_;
};

class CanWriteThread
{
public:
  CanWriteThread(
    const std::string & can_interface, std::array<MotorAxis, 2> & motor_axis,
    const uint8_t number_of_joints)
  : motor_axis_(motor_axis), number_of_joints_(number_of_joints)
  {
    sender_ = drivers::socketcan::SocketCanSender(can_interface);
    thread_ = std::thread([this]() { this->operator()(); });
  }
  void operator()()
  {
    const uint8_t command_id_length = 5;

    for (uint8_t i = 0; i < number_of_joints_; i++) {
      auto & motor_axis = motor_axis_.get().at(i);
      uint16_t can_id =
        static_cast<uint16_t>(static_cast<uint16_t>(motor_axis.GetNodeId()) << command_id_length) |
        static_cast<uint16_t>(motor_axis.GetCommandId());
      (void)can_id;
      // uint64_t data = (motor_axis.GetCommandValue()
    }
  };

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  uint8_t number_of_joints_{};
  std::thread thread_;
  drivers::socketcan::SocketCanSender sender_;
};

class Can
{
public:
  hardware_interface::CallbackReturn Init(
    const std::string & can_interface, std::array<MotorAxis, 2> & motor_axis,
    const uint8_t number_of_joints)
  {
    number_of_joints_ = number_of_joints;
    can_interface_ = can_interface;

    try {
      receiver_ = std::make_unique<CanReadThread>(can_interface_, motor_axis, number_of_joints_);
      sender_ = std::make_unique<CanWriteThread>(can_interface_, motor_axis, number_of_joints_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "Failed to open CAN interface: %s",
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  };

private:
  uint8_t number_of_joints_{};
  std::string can_interface_;
  std::thread read_thread_;
  std::thread write_thread_;
  std::unique_ptr<CanReadThread> receiver_;
  std::unique_ptr<CanWriteThread> sender_;
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_