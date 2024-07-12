#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_can_driver/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace odrive_can_driver
{
hardware_interface::CallbackReturn OdriveHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // If zero or more than two joints then return error, If one then warn, If two then success
  switch (info_.joints.size()) {
    case 1: {
      RCLCPP_WARN(
        rclcpp::get_logger("odrive_hardware_interface"),
        "Expected 2 joints, got 1. One actuator is left uncontrolled!");
      break;
    }
    case 2: {
      // Success, continue with initialization
      break;
    }
    default: {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "Expected 2 joints, got %zu",
        info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (const auto & joint : info_.joints) {
    // TODO
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Setup SocketCan
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION));
  return {};
}

std::vector<hardware_interface::CommandInterface>
OdriveHardwareInterface::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type OdriveHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveHardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return hardware_interface::return_type::OK;
}

}  // namespace odrive_can_driver

PLUGINLIB_EXPORT_CLASS(
  odrive_can_driver::OdriveHardwareInterface, hardware_interface::ActuatorInterface)