#include <array>
#include <cstddef>
#include <cstdint>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_can_driver/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace odrive_can_driver
{

const uint8_t kMaxNodeId = 63;

constexpr std::array<std::string_view, 3> kSupportedInterfaces{
  static_cast<const char *>(hardware_interface::HW_IF_POSITION),
  static_cast<const char *>(hardware_interface::HW_IF_VELOCITY),
  static_cast<const char *>(hardware_interface::HW_IF_EFFORT)};

constexpr std::array<std::pair<std::string_view, CommandId>, 3> kInterfaceToCommandId{
  {{static_cast<const char *>(hardware_interface::HW_IF_POSITION), CommandId::kInputPos},
   {static_cast<const char *>(hardware_interface::HW_IF_VELOCITY), CommandId::kInputVel},
   {static_cast<const char *>(hardware_interface::HW_IF_EFFORT), CommandId::kInputTorque}}};

CommandId InterfaceToCommandId(const std::string_view & interface)
{
  const auto * const interface_command_pair = std::find_if(
    kInterfaceToCommandId.begin(), kInterfaceToCommandId.end(),
    [&interface](const auto & pair) { return pair.first == interface; });
  if (interface_command_pair != kInterfaceToCommandId.end()) {
    return interface_command_pair->second;
  }
  return CommandId::kNoCommand;
}

bool IsSupportedInterface(const std::string & interface)
{
  return kSupportedInterfaces.end() !=
         std::find(kSupportedInterfaces.begin(), kSupportedInterfaces.end(), interface);
}

bool IsPositiveInteger(const std::string & input)
{
  return !input.empty() && std::find_if(input.begin(), input.end(), [](unsigned char digit) {
                             return std::isdigit(digit) == 0;
                           }) == input.end();
}

std::optional<std::string> GetJointInterface(
  const std::vector<std::string> & keys, const std::string & joint_name)
{
  const std::string key_joint_prefix = joint_name + "/";

  for (const auto & key : keys) {
    if (key.rfind(key_joint_prefix, 0) == 0) {
      return key.substr(key_joint_prefix.size());
    }
  }
  return std::nullopt;
}

int NumberOfInterfaces(const std::vector<std::string> & keys, const std::string & joint_name)
{
  int count = 0;
  const std::string key_joint_prefix = joint_name + "/";

  for (const auto & key : keys) {
    if (key.rfind(key_joint_prefix, 0) == 0) {
      ++count;
    }
  }
  return count;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (
    hardware_interface::ActuatorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto can_interface_it = info_.hardware_parameters.find("interface");
  if (can_interface_it == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(rclcpp::get_logger("odrive_hardware_interface"), "No CAN interface specified");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_interface_ = can_interface_it->second;

  number_of_joints_ = info_.joints.size();
  switch (number_of_joints_) {
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
        rclcpp::get_logger("odrive_hardware_interface"), "Expected 2 joints, got %hhu",
        number_of_joints_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (size_t i = 0; i < number_of_joints_; ++i) {
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
    auto & motor_axis = motor_axis_.at(i);
    motor_axis.node_id = static_cast<uint8_t>(node_id);
    motor_axis.joint_name = info_.joints.at(i).name;
  }
  if (number_of_joints_ == 2 && motor_axis_[0].node_id == motor_axis_[1].node_id) {
    RCLCPP_FATAL(rclcpp::get_logger("odrive_hardware_interface"), "Node IDs must be different");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_interface_);
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_interface_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("odrive_hardware_interface"), "Failed to open CAN interface: %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  // Close SocketCan
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Really nothing to do here, power is already disabled in on_deactivate
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Power motors:
  // Set torque to 0
  // Set input mote to torque
  // Set control mode to closed loop
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  // Set control mode to idle
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
  state_interfaces.reserve(static_cast<size_t>(number_of_joints_ * 3));
  for (size_t i = 0; i < number_of_joints_; ++i) {
    auto & motor_axis = motor_axis_.at(i);
    state_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_POSITION, &motor_axis.position_state);
    state_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_VELOCITY, &motor_axis.velocity_state);
    state_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_EFFORT, &motor_axis.effort_state);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OdriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(number_of_joints_);
  for (size_t i = 0; i < number_of_joints_; ++i) {
    auto & motor_axis = motor_axis_.at(i);
    command_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_POSITION, &motor_axis.position_command);
    command_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_VELOCITY, &motor_axis.velocity_command);
    command_interfaces.emplace_back(
      motor_axis.joint_name, hardware_interface::HW_IF_EFFORT, &motor_axis.effort_command);
  }
  return command_interfaces;
}

hardware_interface::return_type OdriveHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  bool error = false;
  for (size_t i = 0; i < number_of_joints_; ++i) {
    auto & motor_axis = motor_axis_.at(i);
    switch (NumberOfInterfaces(start_interfaces, motor_axis.joint_name)) {
      case 0: {
        break;
      }
      case 1: {
        const auto start_interface = GetJointInterface(start_interfaces, motor_axis.joint_name);
        const auto stop_interface = GetJointInterface(stop_interfaces, motor_axis.joint_name);

        // Provided interface must be valid
        error |= !IsSupportedInterface(start_interface.value());

        // 1. Previous interface must be stopped if it's active
        // 2. No interface can be stopped if no interface is active
        error |= (motor_axis.active_command != CommandId::kNoCommand) == stop_interface.has_value();

        // Interface being stopped must be the same as previously active interface
        error |= stop_interface.has_value() &&
                 InterfaceToCommandId(stop_interface.value()) == motor_axis.active_command;

        break;
      }
      default:
        error = true;
    }
  }
  return error ? hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (size_t i = 0; i < number_of_joints_; ++i) {
    auto & motor_axis = motor_axis_.at(i);
    auto start_interface = GetJointInterface(start_interfaces, motor_axis.joint_name);
    if (start_interface.has_value()) {
      motor_axis.active_command = InterfaceToCommandId(start_interface.value());
    } else {
      auto stop_interface = GetJointInterface(stop_interfaces, motor_axis.joint_name);
      if (
        stop_interface.has_value() &&
        InterfaceToCommandId(stop_interface.value()) == motor_axis.active_command) {
        motor_axis.active_command = CommandId::kNoCommand;
        // TODO: disable motor
        // Set torque to 0
        // Set input mote to torque
        // Set control mode to closed loop
      }
    }
  }
  return hardware_interface::return_type::OK;
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

  for (size_t i = 0; i < number_of_joints_; ++i) {
    auto & motor_axis = motor_axis_.at(i);
    if (motor_axis.active_command != CommandId::kNoCommand) {
      if (motor_axis.timeout_error) {
        // send clear errors command
      }
      // send active command
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace odrive_can_driver

PLUGINLIB_EXPORT_CLASS(
  odrive_can_driver::OdriveHardwareInterface, hardware_interface::ActuatorInterface)