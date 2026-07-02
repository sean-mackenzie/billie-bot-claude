#include "billiebot_mission/battery_guard_node.hpp"

namespace billiebot_mission
{

BatteryGuardNode::BatteryGuardNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList BatteryGuardNode::providedPorts()
{
  return {
    BT::InputPort<double>("voltage", 12.6, "Current battery voltage"),
    BT::InputPort<double>("threshold", 10.5,
                           "Minimum safe voltage (3.5V/cell * 3)"),
  };
}

BT::NodeStatus BatteryGuardNode::tick()
{
  double voltage = 12.6;
  double threshold = 10.5;

  getInput("voltage", voltage);
  getInput("threshold", threshold);

  if (voltage < threshold) {
    // Battery low — trigger SAFE mode
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace billiebot_mission
