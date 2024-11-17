#ifndef ODRIVE_CAN_DRIVER_CAN_PARSING_H_
#define ODRIVE_CAN_DRIVER_CAN_PARSING_H_

#include <sys/types.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
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

template <typename T>
constexpr auto FromUnsignedBuffer(UnsignedEquivalent<T> value)
{
  if constexpr (std::is_integral_v<T>) {
    return static_cast<T>(value);
  } else {
    T result;
    std::memcpy(&result, &value, sizeof(value));
    return result;
  }
}

template <typename... T>
constexpr std::array<std::byte, (sizeof(T) + ...)> PackToLittleEndian(const T... args)
{
  std::array<std::byte, (sizeof(T) + ...)> result{};
  size_t result_index = 0;
  auto pack_single = [&result, &result_index ](auto && arg) constexpr
  {
    static_assert(
      std::is_floating_point_v<std::decay_t<decltype(arg)>> ||
        std::is_integral_v<std::decay_t<decltype(arg)>>,
      "Only floating point and integral types (including chars) are supported");
    auto unsigned_arg = ToUnsignedBuffer(arg);
    const uint8_t k_byte_mask = 0xFF;
    for (size_t arg_index = 0; arg_index < sizeof(unsigned_arg); ++arg_index) {
      result.at(result_index) = std::byte((unsigned_arg >> arg_index * 8) & k_byte_mask);
      ++result_index;
    }
  };

  (pack_single(args), ...);

  return result;
}

template <typename... T>
constexpr std::tuple<T...> UnpackFromLittleEndian(
  const std::array<std::byte, (sizeof(T) + ...)> & data)
{
  std::tuple<T...> result;
  size_t data_index = 0;

  auto unpack_single = [&data, &data_index ](auto & arg) constexpr
  {
    static_assert(
      std::is_floating_point_v<std::decay_t<decltype(arg)>> ||
        std::is_integral_v<std::decay_t<decltype(arg)>>,
      "Only floating point and integral types (including chars) are supported");
    using UnsignedBuffer = UnsignedEquivalent<decltype(arg)>;
    UnsignedBuffer unsigned_buffer = 0;

    for (size_t arg_index = 0; arg_index < sizeof(UnsignedBuffer); ++arg_index) {
      unsigned_buffer |= static_cast<UnsignedBuffer>(std::to_integer<uint8_t>(data.at(data_index)))
                         << (arg_index * 8);
      ++data_index;
    }
    arg = FromUnsignedBuffer<std::remove_reference_t<decltype(arg)>>(unsigned_buffer);
  };

  std::apply([&unpack_single](auto &... args) { (unpack_single(args), ...); }, result);

  return result;
}

[[nodiscard]] inline drivers::socketcan::CanId CreateCanId(
  const uint16_t node_id, CommandId command_id, drivers::socketcan::FrameType frame_type,
  uint16_t bus_time = 0)

{
  const uint8_t k_command_id_length = 5;
  return {
    static_cast<uint32_t>(
      static_cast<uint16_t>(node_id << k_command_id_length) | static_cast<uint16_t>(command_id)),
    bus_time, frame_type, drivers::socketcan::StandardFrame};
};

// enum for lifecycle transitions
enum class HardwareState : uint8_t {
  kConfigure = 1,
  kActivate = 2,
  kRun = 3,
  kDeactivate = 4,
  kCleanup = 5,
  kError = 0
};

}  // namespace odrive_can_driver
#endif /* ODRIVE_CAN_DRIVER_CAN_PARSING_H_ */
