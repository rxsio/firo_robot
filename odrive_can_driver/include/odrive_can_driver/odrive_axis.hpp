#ifndef ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#define ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <string>
#include <utility>
namespace odrive_can_driver
{
enum class CommandId : uint8_t {
  kNoCommand = 0x000,
  kMotorError = 0x003,
  kEncoderError = 0x004,
  kAxisRequestedState =
    0x007,  // 1 - idle, 2 - startup sequence, 3 - full calibration, 6 - encoder index search, 8 - closed loop control
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
  AxisAtomicProperty<double> position_state;
  AxisAtomicProperty<double> velocity_state;
  AxisAtomicProperty<double> effort_state;

  AxisAtomicProperty<enum CommandId> command;
  AxisAtomicProperty<double> position_command;
  AxisAtomicProperty<double> velocity_command;
  AxisAtomicProperty<double> effort_command;

  AxisAtomicProperty<bool> timeout_error;
  AxisAtomicProperty<bool> error;

  void Init(const std::string & joint_name, const uint8_t node_id)
  {
    joint_name_ = joint_name;
    node_id_ = node_id;
  }

  [[nodiscard]] auto JointName() const { return joint_name_; }
  [[nodiscard]] auto NodeId() const { return node_id_; }
  [[nodiscard]] double CommandValue() const;

private:
  std::string joint_name_;
  uint8_t node_id_{};
};

class MotorAxisInterface
{
public:
  double position_state{};
  double velocity_state{};
  double effort_state{};

  CommandId command{CommandId::kNoCommand};
  double position_command{};
  double velocity_command{};
  double effort_command{};

  bool timeout_error{false};
  bool error{false};  // Not counting timeout error

  explicit MotorAxisInterface(
    MotorAxis & motor_axis, const double transmission = 1, const double torque_constant = 1)
  : motor_axis_(&motor_axis),
    torque_factor_(torque_constant * (transmission < 0 ? -1 : 1)),
    position_factor_(2 * M_PI / transmission)
  {
  }

  void Read();

  void Write();

  [[nodiscard]] auto JointName() const { return motor_axis_->JointName(); }
  [[nodiscard]] auto NodeId() const { return motor_axis_->NodeId(); }

private:
  MotorAxis * motor_axis_{};
  double torque_factor_{};
  double position_factor_{};
};

}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER_ODRIVE_AXIS_HPP