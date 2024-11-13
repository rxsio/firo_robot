#ifndef ODRIVE_CAN_DRIVER_CAN_CAN_WRITE_THREAD_H_
#define ODRIVE_CAN_DRIVER_CAN_CAN_WRITE_THREAD_H_

#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <hardware_interface/actuator_interface.hpp>
#include <odrive_can_driver/can/parsing.hpp>
#include <odrive_can_driver/odrive_axis.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <ros2_socketcan/socket_can_id.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <stdexcept>
#include <thread>
namespace odrive_can_driver
{

class CanWriteThread
{
public:
  CanWriteThread(const CanWriteThread &) = delete;
  CanWriteThread(CanWriteThread &&) = delete;
  CanWriteThread & operator=(const CanWriteThread &) = delete;
  CanWriteThread & operator=(CanWriteThread &&) = delete;
  CanWriteThread(const std::string & can_interface, std::vector<MotorAxis> & motor_axis)
  : motor_axis_(motor_axis), sender_(can_interface)
  {
    // We can't pass this object directly, because it's not copyable
    // (drivers::socketcan::SocketCanReceiver and other data members are not copyable)
    thread_ = std::thread(std::ref(*this));
  }
  void operator()();
  void Notify(const rclcpp::Time & time, const rclcpp::Duration & period);
  ~CanWriteThread();

private:
  std::reference_wrapper<std::vector<MotorAxis>> motor_axis_;
  std::thread thread_;
  drivers::socketcan::SocketCanSender sender_;
  rclcpp::Time time_{0};
  rclcpp::Duration period_{0, 0};
  std::mutex timestamp_mutex_;
  std::condition_variable wait_for_next_write_;
  std::atomic<bool> run_{true};
  const rclcpp::Duration k_min_timeout_{0, 1000000};

  template <typename... T>
  bool Send(
    const uint8_t node_id, CommandId command, const rclcpp::Time & deadline, const T... data)
  {
    auto packed_data = PackToLittleEndian(data...);

    auto timeout = std::max(k_min_timeout_, deadline - rclcpp::Clock().now());

    try {
      sender_.send(
        static_cast<void *>(packed_data.data()), sizeof(packed_data),
        CreateCanId(node_id, command, drivers::socketcan::FrameType::DATA),
        timeout.to_chrono<std::chrono::nanoseconds>());
      return true;
    } catch (const drivers::socketcan::SocketCanTimeout & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("odrive_hardware_interface"),
        "Error sending CAN message: %hhu, %hhu, %s", node_id, static_cast<uint8_t>(command),
        e.what());
      return false;
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("odrive_hardware_interface"),
        "Error sending CAN message: %hhu, %hhu, %s", node_id, static_cast<uint8_t>(command),
        e.what());
      return false;
    } catch (const std::domain_error & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("odrive_hardware_interface"),
        "Error sending CAN message: %hhu, %hhu, %s", node_id, static_cast<uint8_t>(command),
        e.what());
      return false;
    }
  }
  bool Send(const uint8_t node_id, CommandId command, const rclcpp::Time & deadline);
  void Write(const rclcpp::Time & deadline);
};

}  // namespace odrive_can_driver
#endif /* ODRIVE_CAN_DRIVER_CAN_CAN_WRITE_THREAD_H_ */
