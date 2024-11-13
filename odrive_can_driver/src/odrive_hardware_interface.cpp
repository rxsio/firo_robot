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

  motor_axis_ = std::make_unique<std::vector<MotorAxis>>(info_.joints.size());

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
    // Get transmission and torque constant
    double transmission = 1;
    auto transmission_it = hardware_info.joints.at(i).parameters.find("transmission");
    if (transmission_it != hardware_info.joints.at(i).parameters.end()) {
      try {
        transmission = std::stod(transmission_it->second);
      } catch (const std::invalid_argument & ia) {
        RCLCPP_FATAL(
          rclcpp::get_logger("odrive_hardware_interface"), "Transmission is not a number: %s",
          transmission_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      } catch (const std::out_of_range & oor) {
        RCLCPP_FATAL(
          rclcpp::get_logger("odrive_hardware_interface"), "Transmission is out of range: %s",
          transmission_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    double torque_constant = 1;
    auto torque_constant_it = hardware_info.joints.at(i).parameters.find("torque_constant");
    if (torque_constant_it != hardware_info.joints.at(i).parameters.end()) {
      try {
        torque_constant = std::stod(torque_constant_it->second);
      } catch (const std::invalid_argument & ia) {
        RCLCPP_FATAL(
          rclcpp::get_logger("odrive_hardware_interface"), "Torque constant is not a number: %s",
          torque_constant_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      } catch (const std::out_of_range & oor) {
        RCLCPP_FATAL(
          rclcpp::get_logger("odrive_hardware_interface"), "Torque constant is out of range: %s",
          torque_constant_it->second.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    motor_axis_->at(i).Init(hardware_info.joints.at(i).name, static_cast<uint8_t>(node_id));
    motor_axis_interface_.emplace_back(motor_axis_->at(i), transmission, torque_constant);
  }
  // Node IDs must be different
  // if (std::unique(motor_axis_.begin(), motor_axis_.end(), [](const auto & lhs, const auto & rhs) {
  //       return lhs.GetNodeId() == rhs.GetNodeId();
  //     }) != motor_axis_.end()) {
  //   RCLCPP_FATAL(rclcpp::get_logger("odrive_hardware_interface"), "Node IDs must be different");
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto can_interface = info_.hardware_parameters.find("interface")->second;
  return can_.Init(can_interface, *motor_axis_);
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_.Shutdown();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & axis_interface : motor_axis_interface_) {
    axis_interface.command = CommandId::kNoCommand;
    axis_interface.Write();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OdriveHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OdriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(static_cast<size_t>(motor_axis_interface_.size() * 3));
  for (auto & axis_interface : motor_axis_interface_) {
    state_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_POSITION,
      &axis_interface.position_state);
    state_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_VELOCITY,
      &axis_interface.velocity_state);
    state_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_EFFORT, &axis_interface.effort_state);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OdriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(static_cast<size_t>(motor_axis_interface_.size() * 3));
  for (auto & axis_interface : motor_axis_interface_) {
    command_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_POSITION,
      &axis_interface.position_command);
    command_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_VELOCITY,
      &axis_interface.velocity_command);
    command_interfaces.emplace_back(
      axis_interface.JointName(), hardware_interface::HW_IF_EFFORT, &axis_interface.effort_command);
  }
  return command_interfaces;
}

hardware_interface::return_type OdriveHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  bool error = false;
  for (auto & axis_interface : motor_axis_interface_) {
    const auto joint_name = axis_interface.JointName();
    switch (NumberOfInterfaces(start_interfaces, joint_name)) {
      case 0: {
        break;
      }
      case 1: {
        const auto start_interface = GetJointInterface(start_interfaces, joint_name);
        const auto stop_interface = GetJointInterface(stop_interfaces, joint_name);

        // Provided interface must be valid
        error |= !IsSupportedInterface(start_interface.value());

        // 1. Previous interface must be stopped if it's active
        // 2. No interface can be stopped if no interface is active
        error |= (axis_interface.command == CommandId::kNoCommand) == stop_interface.has_value();

        // Interface being stopped must be the same as previously active interface
        error |= stop_interface.has_value() &&
                 InterfaceToCommandId(stop_interface.value()) == axis_interface.command;

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
  for (auto & axis_interface : motor_axis_interface_) {
    auto start_interface = GetJointInterface(start_interfaces, axis_interface.JointName());
    if (start_interface.has_value()) {
      axis_interface.command = InterfaceToCommandId(start_interface.value());
      axis_interface.Write();
    } else {
      auto stop_interface = GetJointInterface(stop_interfaces, axis_interface.JointName());
      if (
        stop_interface.has_value() &&
        InterfaceToCommandId(stop_interface.value()) == axis_interface.command) {
        axis_interface.command = CommandId::kNoCommand;
        axis_interface.Write();
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  can_.Read(rclcpp::Clock().now(), period);
  for (auto & axis_interface : motor_axis_interface_) {
    axis_interface.Read();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OdriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (auto & axis_interface : motor_axis_interface_) {
    axis_interface.Write();
  }
  can_.Write(rclcpp::Clock().now(), period);
  return hardware_interface::return_type::OK;
}
}  // namespace odrive_can_driver

PLUGINLIB_EXPORT_CLASS(
  odrive_can_driver::OdriveHardwareInterface, hardware_interface::ActuatorInterface)