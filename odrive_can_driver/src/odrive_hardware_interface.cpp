#include <cstddef>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_can_driver/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace odrive_can_driver
{

bool IsPositiveInteger(const std::string & input)
{
  return !input.empty() && std::find_if(input.begin(), input.end(), [](unsigned char digit) {
                             return std::isdigit(digit) == 0;
                           }) == input.end();
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

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

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    auto input_node_id_it = hardware_info.joints.at(i).parameters.find("node_id");
    if (input_node_id_it == hardware_info.joints.at(i).parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "node_id not specified for joint %zu", i);
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto input_node_id = input_node_id_it->second;
    if (!IsPositiveInteger(input_node_id)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"), "Node ID is not a number: %s",
        input_node_id.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    int node_id = std::stoi(input_node_id);
    if (node_id < 0 || node_id > kMaxNodeId || input_node_id.size() > 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("odrive_hardware_interface"),
        "Node ID can only be in range 0-%d, got %s", kMaxNodeId, input_node_id.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    node_id_.at(i) = static_cast<uint8_t>(node_id);
  }
  if (info_.joints.size() == 2 && node_id_.at(0) == node_id_.at(1)) {
    RCLCPP_FATAL(rclcpp::get_logger("odrive_hardware_interface"), "Node IDs must be different");
    return hardware_interface::CallbackReturn::ERROR;
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
  state_interfaces.reserve(info_.joints.size() * 3);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &position_state_.at(i));
    state_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &velocity_state_.at(i));
    state_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &effort_state_.at(i));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OdriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &position_command_.at(i));
    command_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &velocity_command_.at(i));
    command_interfaces.emplace_back(
      info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &effort_command_.at(i));
  }
  return command_interfaces;
}
hardware_interface::return_type OdriveHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
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