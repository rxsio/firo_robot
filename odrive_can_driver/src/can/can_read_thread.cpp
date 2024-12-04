
#include <algorithm>
#include <cstddef>
#include <odrive_can_driver/can/can_read_thread.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <stdexcept>
namespace odrive_can_driver
{

[[nodiscard]] static std::tuple<uint8_t, CommandId> ParseCanId(
  const drivers::socketcan::CanId & can_id)
{
  const uint8_t k_command_id_length = 5;
  auto can_id_base = can_id.identifier();
  const uint8_t node_id = can_id_base >> k_command_id_length;
  const auto command_id = static_cast<CommandId>(can_id_base & 0b11111U);
  return {node_id, command_id};
}

void CanReadThread::Receive(const rclcpp::Time & deadline)
{
  std::array<std::byte, 8> data{};
  drivers::socketcan::CanId can_id;

  auto timeout = std::max(
    k_min_timeout_.to_chrono<std::chrono::nanoseconds>(), CanReadThread::GetTimeout(deadline));
  try {
    can_id = receiver_.receive(static_cast<void *>(data.data()), timeout);
  } catch (const drivers::socketcan::SocketCanTimeout & e) {
    // TODO
    return;
  } catch (const std::runtime_error & e) {
    // TODO
    return;
  } catch (const std::domain_error & e) {
    // TODO
    return;
  }
  if (can_id.frame_type() != drivers::socketcan::FrameType::DATA) {
    return;
  }
  unsigned char node_id = 0;
  CommandId command_id = CommandId::kNoCommand;
  std::tie(node_id, command_id) = ParseCanId(can_id);
  auto motor_axis_it = std::find_if(
    motor_axis_.get().begin(), motor_axis_.get().end(),
    [node_id](const auto & motor_axis) { return motor_axis.NodeId() == node_id; });
  if (motor_axis_it == motor_axis_.get().end()) {
    return;
  }
  auto & motor_axis = *motor_axis_it;

  auto length = can_id.length();
  switch (command_id) {
    case CommandId::kEncoderEstimates: {
      odrive_can_driver::Receive<CommandId::kEncoderEstimates>(data, length, motor_axis);
      break;
    }
    case CommandId::kIq: {
      odrive_can_driver::Receive<CommandId::kIq>(data, length, motor_axis);
      break;
    }
    case CommandId::kControllerError: {
      odrive_can_driver::Receive<CommandId::kControllerError>(data, length, motor_axis);
      break;
    }
    case CommandId::kMotorError: {
      odrive_can_driver::Receive<CommandId::kMotorError>(data, length, motor_axis);
      break;
    }
    case CommandId::kEncoderError: {
      odrive_can_driver::Receive<CommandId::kEncoderError>(data, length, motor_axis);
      break;
    }
    default:
      break;
  }
};

CanReadThread::~CanReadThread()
{
  run_.store(false, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CanReadThread::operator()()
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
      case HardwareState::kRun: {
        Read(time, period);
        break;
      }
      case HardwareState::kError: {
        Error(time);
        break;
      }
      default:
        break;
    }
  }
}

void CanReadThread::Notify(
  const rclcpp::Time & time, const rclcpp::Duration & period, HardwareState state)
{
  std::lock_guard<std::mutex> lock(step_mutex_);
  time_ = time;
  period_ = period;
  state_ = state;
  step_.notify_one();
}
void CanReadThread::Notify(const rclcpp::Time & time, HardwareState state)
{
  std::lock_guard<std::mutex> lock(step_mutex_);
  time_ = time;
  period_ = rclcpp::Duration(0, 0);
  state_ = state;
  step_.notify_one();
}
std::pair<rclcpp::Time, rclcpp::Duration> CanReadThread::Wait(const rclcpp::Time & previous_time)
{
  std::unique_lock<std::mutex> lock(step_mutex_);
  step_.wait(lock, [this, &previous_time]() {
    return !run_.load(std::memory_order_relaxed) || time_ != previous_time;
  });
  return std::make_pair(time_, period_);
}

void CanReadThread::Configure(const rclcpp::Time & time)
{
  for (auto & motor_axis : motor_axis_.get()) {
    const auto node_id = motor_axis.NodeId();
    SendRTR(node_id, CommandId::kControllerError, time);
    SendRTR(node_id, CommandId::kMotorError, time);
    SendRTR(node_id, CommandId::kEncoderError, time);
  }
  while (time == Time().first) {
    Receive(time);
  }
}

void CanReadThread::Read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto deadline = time + period;
  for (auto & motor_axis : motor_axis_.get()) {
    const auto node_id = motor_axis.NodeId();
    SendRTR(node_id, CommandId::kEncoderEstimates, deadline);
    SendRTR(node_id, CommandId::kIq, deadline);
    SendRTR(node_id, CommandId::kControllerError, deadline);
    SendRTR(node_id, CommandId::kMotorError, deadline);
    SendRTR(node_id, CommandId::kEncoderError, deadline);
  }
  while (deadline > rclcpp::Clock().now() && time == Time().first) {
    Receive(deadline);
  }
}

void CanReadThread::Error(const rclcpp::Time & /*time*/) {}

void CanReadThread::SendRTR(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline)
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
  } catch (const std::domain_error & e) {
    // TODO
    return;
  }
}

template <>
void Receive<odrive_can_driver::CommandId::kEncoderEstimates>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 8) {
    auto [encoder_pos, encoder_vel] = UnpackFromLittleEndian<float, float>(data);
    motor_axis.position_state = encoder_pos;
    motor_axis.velocity_state = encoder_vel;
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kIq>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 8) {
    auto [iq_setpoint, iq] = UnpackFromLittleEndian<float, float>(data);
    motor_axis.effort_state = iq;
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kControllerError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.error = true;
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kMotorError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.error = true;
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kEncoderError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.error = true;
  }
};

}  // namespace odrive_can_driver