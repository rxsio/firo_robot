
#include <algorithm>
#include <odrive_can_driver/odrive_axis.hpp>
namespace odrive_can_driver
{
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

}  // namespace odrive_can_driver