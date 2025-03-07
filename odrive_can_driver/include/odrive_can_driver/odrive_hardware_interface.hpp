
#ifndef ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H_
#define ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H_

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/sensor.hpp>
#include <memory>
#include <odrive_can_driver/can/can.hpp>
#include <odrive_can_driver/odrive_axis.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <vector>

namespace odrive_can_driver
{
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

private:
  std::unique_ptr<std::vector<MotorAxis>> motor_axis_;
  std::vector<MotorAxisInterface> motor_axis_interface_;
  Can can_;
};

}  // namespace odrive_can_driver

#endif /* ODRIVE_CAN_DRIVER_ODRIVE_HARDWARE_INTERFACE_H_ */
