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

// Atomic wrapper for MotorAxis internal use
// - Not intended for use outside MotorAxis class
// - Implements only necessary methods and overloads
// - Non-copyable and non-movable, like atomic<T>
// - Allows assignment of T values with specified memory ordering for load/store
// - Default memory ordering is relaxed
template <typename T, std::memory_order M = std::memory_order_relaxed>
class AxisAtomicProperty
{
public:
  AxisAtomicProperty() = default;
  explicit AxisAtomicProperty(T value) : value_(value) {}
  ~AxisAtomicProperty() = default;

  // Delete copy/move constructors and assignment operators
  AxisAtomicProperty(const AxisAtomicProperty &) = delete;
  AxisAtomicProperty & operator=(const AxisAtomicProperty &) = delete;
  AxisAtomicProperty(AxisAtomicProperty &&) = delete;
  AxisAtomicProperty & operator=(AxisAtomicProperty &&) = delete;

  // Member functions use small capitals to mimic atomic<T> interface
  // NOLINTNEXTLINE(readability-identifier-naming)
  [[nodiscard]] T load(std::memory_order order = M) const { return value_.load(order); }
  // NOLINTNEXTLINE(readability-identifier-naming)
  void store(T value, std::memory_order order = M) { value_.store(value, order); }

  AxisAtomicProperty & operator=(T value)
  {
    store(value);
    return *this;
  }
  explicit operator T() const { return load(); }

private:
  std::atomic<T> value_{};
};

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

  AxisAtomicProperty<double> position_state;
  AxisAtomicProperty<double> velocity_state;
  AxisAtomicProperty<double> effort_state;

  AxisAtomicProperty<enum CommandId> command;
  AxisAtomicProperty<double> position_command;
  AxisAtomicProperty<double> velocity_command;
  AxisAtomicProperty<double> effort_command;

  AxisAtomicProperty<bool> timeout_error;
  AxisAtomicProperty<bool> error;

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
      position_state.load(std::memory_order_relaxed) * 2 * M_PI / transmission_;
  }
  void UpdateVelocityState()
  {
    velocity_state_cache =
      velocity_state.load(std::memory_order_relaxed) * 2 * M_PI / transmission_;
  }
  void UpdateEffortState()
  {
    effort_state_cache =
      effort_state.load(std::memory_order_relaxed) * torque_constant_ * torque_direction_;
  }
  void UpdateCommand() { command.store(command_cache, std::memory_order_relaxed); }
  void UpdatePositionCommand()
  {
    position_command.store(
      position_command_cache * transmission_ / (2 * M_PI), std::memory_order_relaxed);
  }
  void UpdateVelocityCommand()
  {
    velocity_command.store(
      velocity_command_cache * transmission_ / (2 * M_PI), std::memory_order_relaxed);
  }
  void UpdateEffortCommand()
  {
    effort_command.store(
      effort_command_cache / (torque_constant_ * torque_direction_), std::memory_order_relaxed);
  }
  void UpdateTimeoutError() { timeout_error_cache = timeout_error.load(std::memory_order_relaxed); }
  void UpdateError() { error_cache = error.load(std::memory_order_relaxed); }

  [[nodiscard]] auto CommandValue() const
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

private:
  std::string joint_name_;
  uint8_t node_id_{};
  double transmission_{};
  double torque_constant_{};
  int8_t torque_direction_{};
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP