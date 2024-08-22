#ifndef ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_
#define ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_

#include <sys/types.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <memory>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>
#include <type_traits>
namespace odrive_can_driver
{

template <typename T>
class SlidingMinMedianMax
{
public:
  explicit SlidingMinMedianMax(std::size_t window_size) : window_(window_size) {}
  void PushBack(const T & duration)
  {
    window_[old_index_] = duration;
    old_index_ = (old_index_ + 1) % window_.size();
  }
  T Max() const { return *std::max_element(window_.begin(), window_.end()); }
  T Min() const { return *std::min_element(window_.begin(), window_.end()); }
  T Median() const
  {
    auto window = window_;
    std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
    return window[window.size() / 2];
  }

private:
  std::vector<T> window_;
  std::size_t old_index_ = 0;
};

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

[[nodiscard]] static drivers::socketcan::CanId CreateCanId(
  const uint16_t node_id, CommandId command_id, drivers::socketcan::FrameType frame_type,
  uint16_t bus_time = 0)

{
  const uint8_t k_command_id_length = 5;
  return {
    static_cast<uint32_t>(
      static_cast<uint16_t>(node_id << k_command_id_length) | static_cast<uint16_t>(command_id)),
    bus_time, frame_type, drivers::socketcan::StandardFrame};
};

class CanReadThread
{
public:
  CanReadThread(const CanReadThread &) = delete;
  CanReadThread(CanReadThread &&) = delete;
  CanReadThread & operator=(const CanReadThread &) = delete;
  CanReadThread & operator=(CanReadThread &&) = delete;
  CanReadThread(
    const std::string & can_interface, std::array<MotorAxis, 2> & motor_axis,
    const uint8_t number_of_joints)
  : motor_axis_(motor_axis), number_of_joints_(number_of_joints)
  {
    receiver_ = drivers::socketcan::SocketCanReceiver(can_interface);
    sender_ = drivers::socketcan::SocketCanSender(can_interface);
    thread_ = std::thread([this]() { this->operator()(); });
  }
  ~CanReadThread()
  {
    run_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  void operator()()
  {
    // SlidingMinMedianMax<rclcpp::Duration> response_max_delay{10};
    while (rclcpp::ok() && run_.load(std::memory_order_relaxed)) {
      auto deadline = GetDeadline();
      for (uint8_t i = 0; i < number_of_joints_; i++) {
        auto & motor_axis = motor_axis_.get().at(i);
        const auto node_id = motor_axis.GetNodeId();
        SendRTR(node_id, CommandId::kEncoderEstimates, deadline);
        SendRTR(node_id, CommandId::kIq, deadline);
        SendRTR(node_id, CommandId::kControllerError, deadline);
        SendRTR(node_id, CommandId::kMotorError, deadline);
        SendRTR(node_id, CommandId::kEncoderError, deadline);
      }
      while (deadline > rclcpp::Clock().now()) {
        Receive(deadline);
      }
    }
  };

  void Notify(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    time_ = time;
    period_ = period;
  }

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  uint8_t number_of_joints_{};
  std::thread thread_;
  drivers::socketcan::SocketCanReceiver receiver_;
  drivers::socketcan::SocketCanSender sender_;
  rclcpp::Time time_{0};
  rclcpp::Duration period_{0, 0};
  rclcpp::Time last_response_time_{0, 0};
  std::mutex timestamp_mutex_;
  std::atomic<bool> run_{true};
  const rclcpp::Duration k_min_timeout_{0, 1000000};

  void SendRTR(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline)
  {
    auto timeout =
      std::max(k_min_timeout_.to_chrono<std::chrono::nanoseconds>(), GetTimeout(deadline));
    try {
      sender_.send(
        std::nullptr_t(), 0, CreateCanId(node_id, command, drivers::socketcan::FrameType::REMOTE),
        timeout);
    } catch (const drivers::socketcan::SocketCanTimeout & e) {
      // TODO
      return;
    } catch (const std::runtime_error & e) {
      // TODO
      return;
    }
  }
  rclcpp::Time GetDeadline()
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    return time_ + period_;
  }
  static std::chrono::nanoseconds GetTimeout(const rclcpp::Time & deadline)
  {
    return (deadline - rclcpp::Clock().now()).to_chrono<std::chrono::nanoseconds>();
  }
  void Receive(const rclcpp::Time & deadline);
};

template <odrive_can_driver::CommandId C>
void Receive(std::array<std::byte, 8> /*data*/, uint32_t /*length*/, MotorAxis & /*motor_axis*/){};
template <>
void Receive<odrive_can_driver::CommandId::kEncoderEstimates>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis);
template <>
void Receive<odrive_can_driver::CommandId::kIq>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis);
template <>
void Receive<odrive_can_driver::CommandId::kControllerError>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis);
template <>
void Receive<odrive_can_driver::CommandId::kMotorError>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis);
template <>
void Receive<odrive_can_driver::CommandId::kEncoderError>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis);

class CanWriteThread
{
public:
  CanWriteThread(const CanWriteThread &) = delete;
  CanWriteThread(CanWriteThread &&) = delete;
  CanWriteThread & operator=(const CanWriteThread &) = delete;
  CanWriteThread & operator=(CanWriteThread &&) = delete;
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
    // TODO: Parametrize init timeout duration
    auto deadline = rclcpp::Clock().now() + rclcpp::Duration(5, 0);
    // TODO: Synchronize with read loop so initial errors are either read first
    // or they are not read at all before initial ClearErrors
    for (auto & motor_axis : motor_axis_.get()) {
      auto node_id = motor_axis.GetNodeId();
      Send(node_id, CommandId::kClearErrors, deadline);
    }
    while (rclcpp::ok() && run_.load(std::memory_order_relaxed)) {
      std::unique_lock<std::mutex> lock(timestamp_mutex_);
      wait_for_next_write_.wait(lock);
      Write(time_ + period_);
    }
    // TODO: Parametrize cleanup timeout duration
    deadline = rclcpp::Clock().now() + rclcpp::Duration(5, 0);
    for (auto & motor_axis : motor_axis_.get()) {
      auto node_id = motor_axis.GetNodeId();
      Send(node_id, CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(0));
      Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(1));
    }
  };
  void Notify(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    std::lock_guard<std::mutex> lock(timestamp_mutex_);
    time_ = time;
    period_ = period;
    wait_for_next_write_.notify_one();
  }
  ~CanWriteThread()
  {
    run_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  std::reference_wrapper<std::array<MotorAxis, 2>> motor_axis_;
  uint8_t number_of_joints_{};
  std::thread thread_;
  drivers::socketcan::SocketCanSender sender_;
  rclcpp::Time time_{0};
  rclcpp::Duration period_{0, 0};
  std::mutex timestamp_mutex_;
  std::condition_variable wait_for_next_write_;
  std::atomic<bool> run_{true};
  const rclcpp::Duration k_min_timeout_{0, 1000000};

  template <typename... T>
  void Send(
    const uint8_t node_id, CommandId command, const rclcpp::Time & deadline, const T... data)
  {
    auto packed_data = PackToLittleEndian(data...);

    try {
      auto timeout = std::max(k_min_timeout_, deadline - rclcpp::Clock().now());
      sender_.send(
        static_cast<void *>(packed_data.data()), sizeof(packed_data),
        CreateCanId(node_id, command, drivers::socketcan::FrameType::DATA),
        timeout.to_chrono<std::chrono::nanoseconds>());
    } catch (const drivers::socketcan::SocketCanTimeout & e) {
      // TODO
      return;
    } catch (const std::runtime_error & e) {
      // TODO
      return;
    }
  }
  void Send(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline)
  {
    auto timeout = std::max(k_min_timeout_, deadline - rclcpp::Clock().now());
    try {
      sender_.send(
        nullptr, 0, CreateCanId(node_id, command, drivers::socketcan::FrameType::DATA),
        timeout.to_chrono<std::chrono::nanoseconds>());
    } catch (const drivers::socketcan::SocketCanTimeout & e) {
      // TODO
      return;
    } catch (const std::runtime_error & e) {
      // TODO
      return;
    }
  }
  void Write(const rclcpp::Time & deadline)
  {
    for (uint8_t i = 0; i < number_of_joints_; i++) {
      auto & motor_axis = motor_axis_.get().at(i);
      auto command_id = motor_axis.GetCommandId();
      const auto node_id = motor_axis.GetNodeId();
      switch (command_id) {
        case CommandId::kInputTorque: {
          Send(node_id, command_id, deadline, motor_axis.GetCommandValue());
          // TODO: Replace command and input modes with enums
          Send(node_id, CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(6));
          Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(8));
          break;
        }
        case CommandId::kInputVel: {
          Send(node_id, command_id, deadline, motor_axis.GetCommandValue(), uint32_t(0));
          Send(node_id, CommandId::kControllerModes, deadline, uint32_t(2), uint32_t(2));
          Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(8));
          break;
        }
        case CommandId::kInputPos: {
          Send(node_id, command_id, deadline, motor_axis.GetCommandValue(), uint32_t(0));
          Send(node_id, CommandId::kControllerModes, deadline, uint32_t(3), uint32_t(1));
          Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(8));
          break;
        }
        default: {
          Send(node_id, CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(0));
          Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(1));
          break;
        }
      }
      if (motor_axis.GetTimeoutError() && !motor_axis.GetError()) {
        Send(node_id, CommandId::kClearErrors, deadline);
      }
    }
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
  hardware_interface::CallbackReturn Shutdown()
  {
    receiver_.reset();
    sender_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  };
  void Write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    sender_->Notify(time, period);
  };

  void Read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    receiver_->Notify(time, period);
  };

private:
  uint8_t number_of_joints_{};
  std::string can_interface_;
  std::unique_ptr<CanReadThread> receiver_;
  std::unique_ptr<CanWriteThread> sender_;
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_