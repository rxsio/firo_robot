
#include <algorithm>
#include <limits>
#include <odrive_can_driver/odrive_axis.hpp>
namespace odrive_can_driver
{
CommandId InterfaceToCommandId(const std::string_view & interface)
{
  const auto * const interface_command_pair = std::find_if(
    kInterfaceToCommandId.begin(), kInterfaceToCommandId.end(),
    [&interface](const auto & pair) { return pair.first == interface; });
  if (interface_command_pair != kInterfaceToCommandId.end()) {
    return interface_command_pair->second;
  }
  return CommandId::kNoCommand;
}
bool IsSupportedInterface(const std::string & interface)
{
  return kSupportedInterfaces.end() !=
         std::find(kSupportedInterfaces.begin(), kSupportedInterfaces.end(), interface);
}

[[nodiscard]] double MotorAxis::CommandValue() const
{
  switch (command.load()) {
    case CommandId::kInputPos:
      return position_command.load();
    case CommandId::kInputVel:
      return velocity_command.load();
    case CommandId::kInputTorque:
      return effort_command.load();
    default:
      // This should never occur
      return std::numeric_limits<double>::quiet_NaN();
  }
}

void MotorAxisInterface::Read()
{
  position_state = motor_axis_->position_state.load() * position_factor_;
  velocity_state = motor_axis_->velocity_state.load() * position_factor_;
  effort_state = motor_axis_->effort_state.load() * torque_factor_;
  timeout_error = motor_axis_->timeout_error.load();
  error = motor_axis_->error.load();
}

void MotorAxisInterface::Write()
{
  motor_axis_->command = command;
  motor_axis_->position_command = position_command / position_factor_;
  motor_axis_->velocity_command = velocity_command / position_factor_;
  motor_axis_->effort_command = effort_command / torque_factor_;
}

}  // namespace odrive_can_driver