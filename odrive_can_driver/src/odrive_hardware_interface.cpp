#include "odrive_can_driver/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_can_driver
{
hardware_interface::CallbackReturn OdriveHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  (void)hardware_info;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
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