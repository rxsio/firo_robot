
#include <odrive_can_driver/can/can.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
namespace odrive_can_driver
{
hardware_interface::CallbackReturn Can::Init(
  const std::string & can_interface, std::vector<MotorAxis> & motor_axis)
{
  try {
    receiver_ = std::make_unique<CanReadThread>(can_interface, motor_axis);
    sender_ = std::make_unique<CanWriteThread>(can_interface, motor_axis);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odrive_hardware_interface"), "Failed to open CAN interface: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
};

}  // namespace odrive_can_driver