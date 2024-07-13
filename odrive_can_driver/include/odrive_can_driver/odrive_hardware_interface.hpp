
#ifndef __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__
#define __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__

#include <array>
#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/sensor.hpp>

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
struct MotorAxis
{
  std::string joint_name;
  uint8_t node_id{0};

  double position_state{0};
  double velocity_state{0};
  double effort_state{0};

  CommandId active_command{CommandId::kNoCommand};
  double position_command{0};
  double velocity_command{0};
  double effort_command{0};

  bool timeout_error{false};
  bool error{false};  // Not counting timeout error
};
class OdriveHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::array<MotorAxis, 2> motor_axis_;
  uint8_t number_of_joints_{0};
};

}  // namespace odrive_can_driver

#endif  // __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__