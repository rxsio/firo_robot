
#ifndef __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__
#define __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/sensor.hpp>

namespace odrive_can_driver
{
class OdriveHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  virtual hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;
  virtual hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
};
}  // namespace odrive_can_driver

#endif  // __ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H__