#include <atomic>
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
  [[nodiscard]] CommandId GetCommand()
  {
    UpdateCommand();
    return command_cache;
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