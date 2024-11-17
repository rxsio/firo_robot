#ifndef ODRIVE_CAN_DRIVER_CAN_CAN_H_
#define ODRIVE_CAN_DRIVER_CAN_CAN_H_

#include <sys/types.h>

#include <hardware_interface/actuator_interface.hpp>
#include <odrive_can_driver/can/can_read_thread.hpp>
#include <odrive_can_driver/can/can_write_thread.hpp>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>

#include "odrive_can_driver/can/parsing.hpp"
namespace odrive_can_driver
{

class Can
{
public:
  hardware_interface::CallbackReturn Init(
    const std::string & can_interface, std::vector<MotorAxis> & motor_axis);
  hardware_interface::CallbackReturn Shutdown()
  {
    receiver_.reset();
    sender_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
  };
  void Configure(const rclcpp::Time & time)
  {
    sender_->Notify(time, HardwareState::kConfigure);
    receiver_->Notify(time, HardwareState::kConfigure);
  };
  void Activate(const rclcpp::Time & time) { sender_->Notify(time, HardwareState::kActivate); };
  void Write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    sender_->Notify(time, period);
  };

  void Read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    receiver_->Notify(time, period);
  };
  void Deactivate(const rclcpp::Time & time) { sender_->Notify(time, HardwareState::kDeactivate); };
  void Cleanup(const rclcpp::Time & time) { sender_->Notify(time, HardwareState::kCleanup); };
  void Error(const rclcpp::Time & time)
  {
    sender_->Notify(time, HardwareState::kError);
    receiver_->Notify(time, HardwareState::kError);
  };

private:
  std::unique_ptr<CanReadThread> receiver_;
  std::unique_ptr<CanWriteThread> sender_;
};
}  // namespace odrive_can_driver
#endif /* ODRIVE_CAN_DRIVER_CAN_CAN_H_ */
