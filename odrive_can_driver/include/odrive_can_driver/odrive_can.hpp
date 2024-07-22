#ifndef ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_
#define ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_

#include <array>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <memory>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/logging.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <thread>
namespace odrive_can_driver
{
class OdriveCan
{
public:
  OdriveCan(const OdriveCan &) = delete;
  OdriveCan(OdriveCan &&) = default;
  OdriveCan & operator=(const OdriveCan &) = delete;
  OdriveCan & operator=(OdriveCan &&) = default;
  explicit OdriveCan(std::array<MotorAxis, 2> & motor_axis)
  : motor_axis_({std::ref(motor_axis[0]), std::ref(motor_axis[1])}){};
  hardware_interface::CallbackReturn Configure(
    const std::string & can_interface, const uint8_t number_of_joints)
  {
    number_of_joints_ = number_of_joints;
    can_interface_ = can_interface;

    try {
      receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_);
      sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_);
      rtr_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_);
      read_thread_ =
        std::thread(&OdriveCan::ReadThread, this, std::ref(*receiver_), std::ref(*rtr_sender_));

      write_thread_ = std::thread(&OdriveCan::WriteThread, this, std::ref(*sender_));
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "Failed to open CAN interface: %s",
        e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    read_thread_ = std::thread(std::ref(*this));
    return hardware_interface::CallbackReturn::SUCCESS;
  };

  ~OdriveCan()
  {
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (write_thread_.joinable()) {
      write_thread_.join();
    }
  };

private:
  std::array<std::reference_wrapper<MotorAxis>, 2> motor_axis_;
  uint8_t number_of_joints_{};
  std::string can_interface_;
  std::thread read_thread_;
  std::thread write_thread_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> rtr_sender_;
  void ReadThread(
    const drivers::socketcan::SocketCanReceiver & /*receiver*/,
    const drivers::socketcan::SocketCanSender & /*rtr_sender*/)
  {
    while (true) {
      (void)motor_axis_;
    }
  };
  void WriteThread(const drivers::socketcan::SocketCanSender & /*sender*/)
  {
    while (true) {
      (void)motor_axis_;
    }
  };
};
}  // namespace odrive_can_driver
#endif  // ODRIVE_CAN_DRIVER__ODRIVE_CAN_HPP_