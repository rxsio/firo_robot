#ifndef ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_
#define ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <memory>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>
#include <type_traits>
namespace odrive_can_driver
{

// Helper types for selecting unsigned integer type based on the size of the input type
// UnsignedOfSize should not be used directly, use UnsignedEquivalent instead
template <size_t T>
struct UnsignedOfSize
{
  static_assert(T > 8, "Type cannot be larger than 8 bytes");
  static_assert(T < 8, "Type size has to be a power of 2");
};
template <>
struct UnsignedOfSize<1>
{
  using Type = uint8_t;
};
template <>
struct UnsignedOfSize<2>
{
  using Type = uint16_t;
};
template <>
struct UnsignedOfSize<4>
{
  using Type = uint32_t;
};
template <>
struct UnsignedOfSize<8>
{
  using Type = uint64_t;
};
template <typename T>
using UnsignedEquivalent = typename UnsignedOfSize<sizeof(T)>::Type;

// Converts given value to an unsigned integer buffer
template <typename T>
constexpr auto ToUnsignedBuffer(const T value)
{
  if constexpr (std::is_integral_v<T>) {
    return static_cast<std::make_unsigned_t<std::decay_t<T>>>(value);
  } else {
    UnsignedEquivalent<T> result;
    std::memcpy(&result, &value, sizeof(value));
    return result;
  }
};

template <typename... T>
constexpr std::array<std::byte, (sizeof(T) + ...)> PackToLittleEndian(const T... args)
{
  std::array<std::byte, (sizeof(T) + ...)> result{};
  size_t result_index = 0;
  auto pack_to_result = [&result, &result_index ](auto && arg) constexpr
  {
    static_assert(
      std::is_floating_point_v<std::decay_t<decltype(arg)>> ||
        std::is_integral_v<std::decay_t<decltype(arg)>>,
      "Only floating point and integral types (including chars) are supported");
    auto unsigned_arg = ToUnsignedBuffer(arg);
    const uint8_t k_byte_mask = 0xFF;
    const uint8_t k_bits_in_byte = 8;
    for (size_t arg_index = 0; arg_index < sizeof(unsigned_arg); ++arg_index) {
      result.at(result_index) =
        std::byte((unsigned_arg >> arg_index * k_bits_in_byte) & k_byte_mask);
      ++result_index;
    }
  };

  (pack_to_result(args), ...);

  return result;
}

class CanID
{
private:
  static const uint8_t kCommandIdLength = 5;

  [[nodiscard]] static drivers::socketcan::CanId CreateCanId(
    const uint16_t node_id, uint16_t bus_time, CommandId command_id,
    drivers::socketcan::FrameType frame_type)
  {
    return {
      static_cast<uint32_t>(
        static_cast<uint16_t>(node_id << kCommandIdLength) | static_cast<uint16_t>(command_id)),
      bus_time, frame_type, drivers::socketcan::StandardFrame};
  }

public:
  CanID() = default;
  explicit CanID(const uint16_t node_id, uint16_t bus_time)
  : motor_error_{CreateCanId(
      node_id, bus_time, CommandId::kMotorError, drivers::socketcan::FrameType::DATA)},
    motor_error_rtr_{CreateCanId(
      node_id, bus_time, CommandId::kMotorError, drivers::socketcan::FrameType::REMOTE)},
    encoder_error_{CreateCanId(
      node_id, bus_time, CommandId::kEncoderError, drivers::socketcan::FrameType::DATA)},
    encoder_error_rtr_{CreateCanId(
      node_id, bus_time, CommandId::kEncoderError, drivers::socketcan::FrameType::REMOTE)},
    axis_requested_state_{CreateCanId(
      node_id, bus_time, CommandId::kAxisRequestedState, drivers::socketcan::FrameType::DATA)},
    encoder_estimates_{CreateCanId(
      node_id, bus_time, CommandId::kEncoderEstimates, drivers::socketcan::FrameType::DATA)},
    encoder_estimates_rtr_{CreateCanId(
      node_id, bus_time, CommandId::kEncoderEstimates, drivers::socketcan::FrameType::REMOTE)},
    controller_modes_{CreateCanId(
      node_id, bus_time, CommandId::kControllerModes, drivers::socketcan::FrameType::DATA)},
    input_pos_{
      CreateCanId(node_id, bus_time, CommandId::kInputPos, drivers::socketcan::FrameType::DATA)},
    input_vel_{
      CreateCanId(node_id, bus_time, CommandId::kInputVel, drivers::socketcan::FrameType::DATA)},
    input_torque_{
      CreateCanId(node_id, bus_time, CommandId::kInputTorque, drivers::socketcan::FrameType::DATA)},
    iq_{CreateCanId(node_id, bus_time, CommandId::kIq, drivers::socketcan::FrameType::DATA)},
    iq_rtr_{CreateCanId(node_id, bus_time, CommandId::kIq, drivers::socketcan::FrameType::REMOTE)},
    reboot_{
      CreateCanId(node_id, bus_time, CommandId::kReboot, drivers::socketcan::FrameType::DATA)},
    clear_errors_{
      CreateCanId(node_id, bus_time, CommandId::kClearErrors, drivers::socketcan::FrameType::DATA)},
    controller_error_{CreateCanId(
      node_id, bus_time, CommandId::kControllerError, drivers::socketcan::FrameType::DATA)},
    controller_error_rtr_{CreateCanId(
      node_id, bus_time, CommandId::kControllerError, drivers::socketcan::FrameType::REMOTE)}
  {
  }

  template <bool RTR = false>
  const drivers::socketcan::CanId & operator[](const CommandId command_id) const
  {
    switch (command_id) {
      case CommandId::kMotorError: {
        return RTR ? motor_error_rtr_ : motor_error_;
      }
      case CommandId::kEncoderError: {
        return RTR ? encoder_error_rtr_ : encoder_error_;
      }
      case CommandId::kAxisRequestedState: {
        return axis_requested_state_;
      }
      case CommandId::kEncoderEstimates: {
        return RTR ? encoder_estimates_rtr_ : encoder_estimates_;
      }
      case CommandId::kControllerModes: {
        return controller_modes_;
      }
      case CommandId::kInputPos: {
        return input_pos_;
      }
      case CommandId::kInputVel: {
        return input_vel_;
      }
      case CommandId::kInputTorque: {
        return input_torque_;
      }
      case CommandId::kIq: {
        return RTR ? iq_rtr_ : iq_;
      }
      case CommandId::kReboot: {
        return reboot_;
      }
      case CommandId::kClearErrors: {
        return clear_errors_;
      }
      case CommandId::kControllerError: {
        return RTR ? controller_error_rtr_ : controller_error_;
      }
      default: {
        throw std::invalid_argument("Invalid command ID");
      }
    };
  }

private:
  drivers::socketcan::CanId motor_error_;
  drivers::socketcan::CanId motor_error_rtr_;
  drivers::socketcan::CanId encoder_error_;
  drivers::socketcan::CanId encoder_error_rtr_;
  drivers::socketcan::CanId axis_requested_state_;
  drivers::socketcan::CanId encoder_estimates_;
  drivers::socketcan::CanId encoder_estimates_rtr_;
  drivers::socketcan::CanId controller_modes_;
  drivers::socketcan::CanId input_pos_;
  drivers::socketcan::CanId input_vel_;
  drivers::socketcan::CanId input_torque_;
  drivers::socketcan::CanId iq_;
  drivers::socketcan::CanId iq_rtr_;
  drivers::socketcan::CanId reboot_;
  drivers::socketcan::CanId clear_errors_;
  drivers::socketcan::CanId controller_error_;
  drivers::socketcan::CanId controller_error_rtr_;
};

class CanReadThread
{
public:
  CanReadThread(
    const std::string & can_interface, std::array<MotorAxis, 2> & motor_axis,
    const std::array<CanID, 2> & can_id, const uint8_t number_of_joints)
  : motor_axis_(motor_axis), can_id_(can_id), number_of_joints_(number_of_joints)
  {
    receiver_ = drivers::socketcan::SocketCanReceiver(can_interface);
    sender_ = drivers::socketcan::SocketCanSender(can_interface);
    thread_ = std::thread([this]() { this->operator()(); });
  }
  void operator()(){
    // auto now = rclcpp::Clock().now();
    // for (uint8_t i = 0; i < number_of_joints_; i++) {
    //   auto & motor_axis = motor_axis_.get().at(i);
    //   const auto & can_id = can_id_.get().at(i);
    // }
  };

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  std::reference_wrapper<const std::array<CanID, 2>> can_id_;
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
    const std::array<CanID, 2> & can_id, const uint8_t number_of_joints)
  : motor_axis_(motor_axis), can_id_(can_id), number_of_joints_(number_of_joints)
  {
    sender_ = drivers::socketcan::SocketCanSender(can_interface);
    thread_ = std::thread([this]() { this->operator()(); });
  }
  void operator()()
  {
    for (uint8_t i = 0; i < number_of_joints_; i++) {
      auto & motor_axis = motor_axis_.get().at(i);
      auto command_id = motor_axis.GetCommandId();
      const auto & can_id = can_id_.get().at(i);
      if (command_id == CommandId::kInputTorque) {
        Send<CommandId::kInputTorque>(can_id, motor_axis.GetCommandValue());
      } else {
        Send<CommandId::kInputPos>(can_id, motor_axis.GetCommandValue(), uint32_t(0));
      }
    }
  };

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  std::reference_wrapper<const std::array<CanID, 2>> can_id_;
  uint8_t number_of_joints_{};
  std::thread thread_;
  drivers::socketcan::SocketCanSender sender_;

  template <CommandId C, typename... T>
  void Send(const odrive_can_driver::CanID & can_id, T... data)
  {
    auto packed_data = PackToLittleEndian(data...);
    sender_.send(static_cast<void *>(packed_data.data()), packed_data.size(), can_id[C]);
  }
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
    can_id_ = {CanID(motor_axis[0].GetNodeId(), 0), CanID(motor_axis[1].GetNodeId(), 0)};

    try {
      receiver_ =
        std::make_unique<CanReadThread>(can_interface_, motor_axis, can_id_, number_of_joints_);
      sender_ =
        std::make_unique<CanWriteThread>(can_interface_, motor_axis, can_id_, number_of_joints_);
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "Failed to open CAN interface: %s",
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  };

private:
  std::array<CanID, 2> can_id_;
  uint8_t number_of_joints_{};
  std::string can_interface_;
  std::thread read_thread_;
  std::thread write_thread_;
  std::unique_ptr<CanReadThread> receiver_;
  std::unique_ptr<CanWriteThread> sender_;
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_