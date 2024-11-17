#include <odrive_can_driver/can/can_write_thread.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <stdexcept>
namespace odrive_can_driver
{
void CanWriteThread::Configure(const rclcpp::Time & time)
{
  // TODO: Parametrize init timeout duration
  auto deadline = time + rclcpp::Duration(5, 0);
  // TODO: Synchronize with read loop so initial errors are either read first
  // or they are not read at all before initial ClearErrors
  for (auto & motor_axis : motor_axis_.get()) {
    auto node_id = motor_axis.NodeId();
    Send(node_id, CommandId::kClearErrors, deadline);
    // Idle
    Send(node_id, CommandId::kAxisRequestedState, deadline, uint32_t(1));
  }
}
void CanWriteThread::Activate(const rclcpp::Time & time)
{
  auto deadline = time + rclcpp::Duration(5, 0);
  for (auto & motor_axis : motor_axis_.get()) {
    // Startup sequence
    Send(motor_axis.NodeId(), CommandId::kAxisRequestedState, deadline, uint32_t(2));
  }
}
void CanWriteThread::Deactivate(const rclcpp::Time & time)
{
  auto deadline = time + rclcpp::Duration(5, 0);
  for (auto & motor_axis : motor_axis_.get()) {
    // Idle
    Send(motor_axis.NodeId(), CommandId::kAxisRequestedState, deadline, uint32_t(1));
  }
}
void CanWriteThread::Cleanup(const rclcpp::Time & time)
{
  // TODO: Parametrize cleanup timeout duration
  auto deadline = time + rclcpp::Duration(5, 0);
  for (auto & motor_axis : motor_axis_.get()) {
    Send(motor_axis.NodeId(), CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(0));
  }
}
void CanWriteThread::Error(const rclcpp::Time & /*time*/) {}

void CanWriteThread::operator()()
{
  rclcpp::Time time;
  rclcpp::Duration period{0, 0};
  while (rclcpp::ok() && run_.load(std::memory_order_relaxed)) {
    std::tie(time, period) = Wait(time);
    switch (state_) {
      case HardwareState::kConfigure: {
        Configure(time);
        break;
      }
      case HardwareState::kActivate: {
        Activate(time);
        break;
      }
      case HardwareState::kRun: {
        Write(time, period);
        break;
      }
      case HardwareState::kDeactivate: {
        Deactivate(time);
        break;
      }
      case HardwareState::kCleanup: {
        Cleanup(time);
        break;
      }
      case HardwareState::kError: {
        Error(time);
        break;
      }
    }
  }
};
void CanWriteThread::Notify(
  const rclcpp::Time & time, const rclcpp::Duration & period, HardwareState state)
{
  std::lock_guard<std::mutex> lock(step_mutex_);
  time_ = time;
  period_ = period;
  state_ = state;
  step_.notify_one();
}
void CanWriteThread::Notify(const rclcpp::Time & time, HardwareState state)
{
  std::lock_guard<std::mutex> lock(step_mutex_);
  time_ = time;
  period_ = rclcpp::Duration(0, 0);
  state_ = state;
  step_.notify_one();
}
std::pair<rclcpp::Time, rclcpp::Duration> CanWriteThread::Wait(const rclcpp::Time & previous_time)
{
  std::unique_lock<std::mutex> lock(step_mutex_);
  step_.wait(lock, [this, &previous_time]() {
    return !run_.load(std::memory_order_relaxed) || time_ != previous_time;
  });
  return std::make_pair(time_, period_);
}
CanWriteThread::~CanWriteThread()
{
  run_.store(false, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool CanWriteThread::Send(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline)
{
  auto timeout = std::max(k_min_timeout_, deadline - rclcpp::Clock().now());
  // Data pointer has still to be valid, because send implementation uses memcpy underneath
  // which is UB for nullptr
  uint8_t data = 0;
  try {
    sender_.send(
      (void *)&data, 0, CreateCanId(node_id, command, drivers::socketcan::FrameType::DATA),
      timeout.to_chrono<std::chrono::nanoseconds>());
    return true;
  } catch (const drivers::socketcan::SocketCanTimeout & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("odrive_hardware_interface"), "Error sending CAN message: %hhu, %hhu, %s",
      node_id, static_cast<uint8_t>(command), e.what());
    return false;
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("odrive_hardware_interface"), "Error sending CAN message: %hhu, %hhu, %s",
      node_id, static_cast<uint8_t>(command), e.what());
    return false;
  } catch (const std::domain_error & e) {
    RCLCPP_WARN(
      rclcpp::get_logger("odrive_hardware_interface"), "Error sending CAN message: %hhu, %hhu, %s",
      node_id, static_cast<uint8_t>(command), e.what());
    return false;
  }
}

void CanWriteThread::Write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto deadline = time + period;
  for (auto & motor_axis : motor_axis_.get()) {
    auto command_id = motor_axis.command.load();
    const auto node_id = motor_axis.NodeId();
    switch (command_id) {
      case CommandId::kInputTorque: {
        Send(node_id, command_id, deadline, float(motor_axis.CommandValue()));
        // TODO: Replace command and input modes with enums
        Send(node_id, CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(6));
        break;
      }
      case CommandId::kInputVel: {
        Send(node_id, command_id, deadline, float(motor_axis.CommandValue()), uint32_t(0));
        Send(node_id, CommandId::kControllerModes, deadline, uint32_t(2), uint32_t(2));
        break;
      }
      case CommandId::kInputPos: {
        Send(node_id, command_id, deadline, float(motor_axis.CommandValue()), uint32_t(0));
        Send(node_id, CommandId::kControllerModes, deadline, uint32_t(3), uint32_t(1));
        break;
      }
      default: {
        Send(node_id, CommandId::kControllerModes, deadline, uint32_t(1), uint32_t(0));
        break;
      }
    }
    if (motor_axis.timeout_error.load() && !motor_axis.error.load()) {
      Send(node_id, CommandId::kClearErrors, deadline);
    }
  }
}
}  // namespace odrive_can_driver