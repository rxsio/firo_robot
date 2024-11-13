
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
  auto [node_id, command_id] = ParseCanId(can_id);
  auto & motor_axis_1 = motor_axis_.get()[0];
  auto & motor_axis_2 = motor_axis_.get()[1];
  auto & motor_axis = motor_axis_1.NodeId() == node_id ? motor_axis_1 : motor_axis_2;
  if (motor_axis.NodeId() != node_id) {
    return;
  }
  auto length = can_id.length();
  switch (command_id) {
    case CommandId::kEncoderEstimates: {
      odrive_can_driver::Receive<CommandId::kEncoderEstimates>(data, length, motor_axis);
      last_response_time_ = rclcpp::Clock().now();
      break;
    }
    case CommandId::kIq: {
      odrive_can_driver::Receive<CommandId::kIq>(data, length, motor_axis);
      last_response_time_ = rclcpp::Clock().now();
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
  while (rclcpp::ok() && run_.load(std::memory_order_relaxed)) {
    std::unique_lock<std::mutex> lock(timestamp_mutex_);
    wait_for_next_read_.wait(lock);
    lock.unlock();
    auto deadline = GetDeadline();
    for (auto & motor_axis : motor_axis_.get()) {
      const auto node_id = motor_axis.NodeId();
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
}

void CanReadThread::Notify(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(timestamp_mutex_);
  time_ = time;
  period_ = period;
  wait_for_next_read_.notify_one();
}

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