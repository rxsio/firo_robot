#ifndef ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#define ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#include <array>
#include <atomic>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <string>
namespace odrive_can_driver
{
enum class CommandId : uint8_t {
  kNoCommand = 0x000,
  kMotorError = 0x003,
  kEncoderError = 0x004,
  kAxisRequestedState = 0x007,
  kEncoderEstimates = 0x009,
  kControllerModes = 0x00B,
  kInputPos = 0x00C,
  kInputVel = 0x00D,
  kInputTorque = 0x00E,
  kIq = 0x014,
  kReboot = 0x016,
  kClearErrors = 0x018,
  kControllerError = 0x01D,
};

constexpr std::array<std::string_view, 3> kSupportedInterfaces{
  static_cast<const char *>(hardware_interface::HW_IF_POSITION),
  static_cast<const char *>(hardware_interface::HW_IF_VELOCITY),
  static_cast<const char *>(hardware_interface::HW_IF_EFFORT)};

constexpr std::array<std::pair<std::string_view, CommandId>, 3> kInterfaceToCommandId{
  {{static_cast<const char *>(hardware_interface::HW_IF_POSITION), CommandId::kInputPos},
   {static_cast<const char *>(hardware_interface::HW_IF_VELOCITY), CommandId::kInputVel},
   {static_cast<const char *>(hardware_interface::HW_IF_EFFORT), CommandId::kInputTorque}}};

CommandId InterfaceToCommandId(const std::string_view & interface);
bool IsSupportedInterface(const std::string & interface);

class MotorAxis
{
public:
  double position_state_cache{};
  double velocity_state_cache{};
  double effort_state_cache{};

  CommandId command_cache{CommandId::kNoCommand};
  double position_command_cache{};
  double velocity_command_cache{};
  double effort_command_cache{};

  bool timeout_error_cache{false};
  bool error_cache{false};  // Not counting timeout error

  void Init(const std::string & joint_name, const uint8_t node_id)
  {
    joint_name_ = joint_name;
    node_id_ = node_id;
  }

  [[nodiscard]] auto GetJointName() const { return joint_name_; }
  [[nodiscard]] auto GetNodeId() const { return node_id_; }

  void UpdatePositionState()
  {
    position_state_cache = position_state_.load(std::memory_order_relaxed);
  }
  void UpdateVelocityState()
  {
    velocity_state_cache = velocity_state_.load(std::memory_order_relaxed);
  }
  void UpdateEffortState() { effort_state_cache = effort_state_.load(std::memory_order_relaxed); }
  void UpdateCommand() { command_cache = command_.load(std::memory_order_relaxed); }
  void UpdatePositionCommand()
  {
    position_command_cache = position_command_.load(std::memory_order_relaxed);
  }
  void UpdateVelocityCommand()
  {
    velocity_command_cache = velocity_command_.load(std::memory_order_relaxed);
  }
  void UpdateEffortCommand()
  {
    effort_command_cache = effort_command_.load(std::memory_order_relaxed);
  }
  void UpdateTimeoutError()
  {
    timeout_error_cache = timeout_error_.load(std::memory_order_relaxed);
  }
  void UpdateError() { error_cache = error_.load(std::memory_order_relaxed); }

  [[nodiscard]] double GetPositionState()
  {
    UpdatePositionState();
    return position_state_cache;
  }
  [[nodiscard]] double GetVelocityState()
  {
    UpdateVelocityState();
    return velocity_state_cache;
  }
  [[nodiscard]] double GetEffortState()
  {
    UpdateEffortState();
    return effort_state_cache;
  }
  [[nodiscard]] CommandId GetCommandId()
  {
    UpdateCommand();
    return command_cache;
  }
  [[nodiscard]] auto GetCommandValue()
  {
    switch (GetCommandId()) {
      case CommandId::kInputPos:
        return GetPositionCommand();
      case CommandId::kInputVel:
        return GetVelocityCommand();
      case CommandId::kInputTorque:
        return GetEffortCommand();
      default:
        // This should never occur
        return std::numeric_limits<double>::quiet_NaN();
    }
  }
  [[nodiscard]] double GetPositionCommand()
  {
    UpdatePositionCommand();
    return position_command_cache;
  }
  [[nodiscard]] double GetVelocityCommand()
  {
    UpdateVelocityCommand();
    return velocity_command_cache;
  }
  [[nodiscard]] double GetEffortCommand()
  {
    UpdateEffortCommand();
    return effort_command_cache;
  }
  [[nodiscard]] bool GetTimeoutError()
  {
    UpdateTimeoutError();
    return timeout_error_cache;
  }
  [[nodiscard]] bool GetError()
  {
    UpdateError();
    return error_cache;
  }

  void SetPositionState(double position)
  {
    position_state_.store(position, std::memory_order_relaxed);
  }
  void SetVelocityState(double velocity)
  {
    velocity_state_.store(velocity, std::memory_order_relaxed);
  }
  void SetEffortState(double effort) { effort_state_.store(effort, std::memory_order_relaxed); }
  void SetCommand(CommandId command) { command_.store(command, std::memory_order_relaxed); }
  void SetPositionCommand(double position)
  {
    position_command_.store(position, std::memory_order_relaxed);
  }
  void SetVelocityCommand(double velocity)
  {
    velocity_command_.store(velocity, std::memory_order_relaxed);
  }
  void SetEffortCommand(double effort) { effort_command_.store(effort, std::memory_order_relaxed); }
  void SetTimeoutError(bool timeout_error)
  {
    timeout_error_.store(timeout_error, std::memory_order_relaxed);
  }
  void SetError(bool error) { error_.store(error, std::memory_order_relaxed); }

private:
  std::string joint_name_;
  uint8_t node_id_{};

  std::atomic<double> position_state_{};
  std::atomic<double> velocity_state_{};
  std::atomic<double> effort_state_{};

  std::atomic<CommandId> command_{};
  std::atomic<double> position_command_{};
  std::atomic<double> velocity_command_{};
  std::atomic<double> effort_command_{};

  std::atomic<bool> timeout_error_{};
  std::atomic<bool> error_{};
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP