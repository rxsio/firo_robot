#ifndef ODRIVE_CAN_DRIVER_CAN_CAN_READ_THREAD_H_
#define ODRIVE_CAN_DRIVER_CAN_CAN_READ_THREAD_H_
// #define ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_

#include <sys/types.h>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <odrive_can_driver/can/parsing.hpp>
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
namespace odrive_can_driver
{

class CanReadThread
{
public:
  CanReadThread(const CanReadThread &) = delete;
  CanReadThread(CanReadThread &&) = delete;
  CanReadThread & operator=(const CanReadThread &) = delete;
  CanReadThread & operator=(CanReadThread &&) = delete;
  CanReadThread(const std::string & can_interface, std::vector<MotorAxis> & motor_axis)
  : motor_axis_(motor_axis), receiver_(can_interface), sender_(can_interface)
  {
    // We can't pass this object directly, because it's not copyable
    // (drivers::socketcan::SocketCanReceiver and other data members are not copyable)
    thread_ = std::thread(std::ref(*this));
  }
  ~CanReadThread();
  void operator()();

  void Notify(
    const rclcpp::Time & time, const rclcpp::Duration & period,
    HardwareState state = HardwareState::kRun);
  void Notify(const rclcpp::Time & time, HardwareState state = HardwareState::kRun);

private:
  std::reference_wrapper<std::vector<MotorAxis>> motor_axis_;
  std::thread thread_;
  drivers::socketcan::SocketCanReceiver receiver_;
  drivers::socketcan::SocketCanSender sender_;
  rclcpp::Time time_{0};
  rclcpp::Duration period_{0, 0};
  std::mutex step_mutex_;
  std::condition_variable step_;
  std::atomic<bool> run_{true};
  const rclcpp::Duration k_min_timeout_{0, 1000000};
  HardwareState state_{};

  void SendRTR(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline);
  void Receive(const rclcpp::Time & deadline);

  std::pair<rclcpp::Time, rclcpp::Duration> Time()
  {
    std::lock_guard<std::mutex> lock(step_mutex_);
    return std::make_pair(time_, period_);
  }

  std::pair<rclcpp::Time, rclcpp::Duration> Wait(const rclcpp::Time & previous_time);

  static std::chrono::nanoseconds GetTimeout(const rclcpp::Time & deadline)
  {
    return (deadline - rclcpp::Clock().now()).to_chrono<std::chrono::nanoseconds>();
  }
  void Configure(const rclcpp::Time & time);
  void Read(const rclcpp::Time & time, const rclcpp::Duration & period);
  void Error(const rclcpp::Time & time);
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

}  // namespace odrive_can_driver
#endif /* ODRIVE_CAN_DRIVER_CAN_CAN_READ_THREAD_H_ */
