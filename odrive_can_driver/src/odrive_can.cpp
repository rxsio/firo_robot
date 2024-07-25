
#include <odrive_can_driver/odrive_can.hpp>
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

void CanReadThread::Receive(std::chrono::nanoseconds timeout)
{
  std::array<std::byte, 8> data{};
  auto can_id = receiver_.receive(static_cast<void *>(data.data()), timeout);
  if (can_id.frame_type() != drivers::socketcan::FrameType::DATA) {
    return;
  }
  auto [node_id, command_id] = ParseCanId(can_id);
  auto & motor_axis_1 = motor_axis_.get()[0];
  auto & motor_axis_2 = motor_axis_.get()[1];
  auto & motor_axis = motor_axis_1.GetNodeId() == node_id ? motor_axis_1 : motor_axis_2;
  if (motor_axis.GetNodeId() != node_id) {
    return;
  }
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

template <>
void Receive<odrive_can_driver::CommandId::kEncoderEstimates>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 8) {
    auto [encoder_pos, encoder_vel] = UnpackFromLittleEndian<float, float>(data);
    motor_axis.SetPositionState(encoder_pos);
    motor_axis.SetVelocityState(encoder_vel);
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kIq>(
  std::array<std::byte, 8> data, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 8) {
    auto [iq_setpoint, iq] = UnpackFromLittleEndian<float, float>(data);
    motor_axis.SetEffortState(iq);
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kControllerError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.SetError(true);
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kMotorError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.SetError(true);
  }
};
template <>
void Receive<odrive_can_driver::CommandId::kEncoderError>(
  std::array<std::byte, 8> /*data*/, uint32_t length, MotorAxis & motor_axis)
{
  if (length == 4) {
    motor_axis.SetError(true);
  }
};

}  // namespace odrive_can_driver