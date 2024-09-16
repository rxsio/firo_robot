#ifndef ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#define ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <string>
#include <utility>
namespace odrive_can_driver
{
enum class CommandId : uint8_t {
  kNoCommand = 0x000,
  kMotorError = 0x003,
  kEncoderError = 0x004,
  kAxisRequestedState = 0x007,  // 1 - idle, 8 - closed loop control
  kEncoderEstimates = 0x009,
  /* kControllerModes valid values for
     Control mode: 1 - Torque, 2 - Velocity, 3 - Position
     Input mode: 0 - Inactive, 1 - Passthrough, 2 - Velocity Ramp, 5 - Trap Trajectory, 6 - Torque Ramp */
  kControllerModes = 0x00B,
  kInputPos = 0x00C,
  kInputVel = 0x00D,
  kInputTorque = 0x00E,
  kIq = 0x014,
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

  MotorAxis() = default;
  explicit MotorAxis(
    std::string joint_name, const uint8_t node_id, const double transmission = 1,
    const double torque_constant = 1)
  : joint_name_(std::move(joint_name)),
    node_id_(node_id),
    transmission_(transmission),
    torque_constant_(torque_constant),
    torque_direction_(transmission_ > 0 ? 1 : -1)
  {
  }
  MotorAxis(MotorAxis &&) = delete;
  MotorAxis & operator=(MotorAxis &&) = delete;
  // Define copy constructor and copy assignment operator
  MotorAxis(const MotorAxis & other)
  : joint_name_(other.joint_name_),
    node_id_(other.node_id_),
    position_state_(other.position_state_.load()),
    velocity_state_(other.velocity_state_.load()),
    effort_state_(other.effort_state_.load()),
    command_(other.command_.load()),
    position_command_(other.position_command_.load()),
    velocity_command_(other.velocity_command_.load()),
    effort_command_(other.effort_command_.load()),
    timeout_error_(other.timeout_error_.load()),
    error_(other.error_.load())
  {
  }
  MotorAxis & operator=(const MotorAxis & other)
  {
    if (this == &other) {
      return *this;
    }
    joint_name_ = other.joint_name_;
    node_id_ = other.node_id_;
    transmission_ = other.transmission_;
    torque_constant_ = other.torque_constant_;
    torque_direction_ = other.torque_direction_;
    position_state_.store(other.position_state_.load());
    velocity_state_.store(other.velocity_state_.load());
    effort_state_.store(other.effort_state_.load());
    command_.store(other.command_.load());
    position_command_.store(other.position_command_.load());
    velocity_command_.store(other.velocity_command_.load());
    effort_command_.store(other.effort_command_.load());
    timeout_error_.store(other.timeout_error_.load());
    error_.store(other.error_.load());
    return *this;
  }

  void Init(
    const std::string & joint_name, const uint8_t node_id, const double transmission = 1,
    const double torque_constant = 1)
  {
    joint_name_ = joint_name;
    node_id_ = node_id;
    transmission_ = transmission;
    torque_constant_ = torque_constant;
    torque_direction_ = transmission_ > 0 ? 1 : -1;
  }

  [[nodiscard]] auto GetJointName() const { return joint_name_; }
  [[nodiscard]] auto GetNodeId() const { return node_id_; }

  void UpdatePositionState()
  {
    position_state_cache =
      position_state_.load(std::memory_order_relaxed) * 2 * M_PI / transmission_;
  }
  void UpdateVelocityState()
  {
    velocity_state_cache =
      velocity_state_.load(std::memory_order_relaxed) * 2 * M_PI / transmission_;
  }
  void UpdateEffortState()
  {
    effort_state_cache =
      effort_state_.load(std::memory_order_relaxed) * torque_constant_ * torque_direction_;
  }
  void UpdateCommand() { command_.store(command_cache, std::memory_order_relaxed); }
  void UpdatePositionCommand()
  {
    position_command_.store(
      position_command_cache * transmission_ / (2 * M_PI), std::memory_order_relaxed);
  }
  void UpdateVelocityCommand()
  {
    velocity_command_.store(
      velocity_command_cache * transmission_ / (2 * M_PI), std::memory_order_relaxed);
  }
  void UpdateEffortCommand()
  {
    effort_command_.store(
      effort_command_cache / (torque_constant_ * torque_direction_), std::memory_order_relaxed);
  }
  void UpdateTimeoutError()
  {
    timeout_error_cache = timeout_error_.load(std::memory_order_relaxed);
  }
  void UpdateError() { error_cache = error_.load(std::memory_order_relaxed); }

  [[nodiscard]] double GetPositionState()
  {
    return position_state_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] double GetVelocityState()
  {
    return velocity_state_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] double GetEffortState() { return effort_state_.load(std::memory_order_relaxed); }
  [[nodiscard]] CommandId GetCommandId() { return command_.load(std::memory_order_relaxed); }
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
    return position_command_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] double GetVelocityCommand()
  {
    return velocity_command_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] double GetEffortCommand()
  {
    return effort_command_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] bool GetTimeoutError() { return timeout_error_.load(std::memory_order_relaxed); }
  [[nodiscard]] bool GetError() { return error_.load(std::memory_order_relaxed); }

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
  double transmission_{};
  double torque_constant_{};
  int8_t torque_direction_{};

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