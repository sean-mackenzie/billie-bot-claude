#include "billiebot_mission/estop_guard_node.hpp"

namespace billiebot_mission
{

EStopGuardNode::EStopGuardNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList EStopGuardNode::providedPorts()
{
  return {
    BT::InputPort<bool>("estopped", false, "Whether e-stop is engaged"),
  };
}

BT::NodeStatus EStopGuardNode::tick()
{
  bool estopped = false;
  getInput("estopped", estopped);

  if (estopped) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace billiebot_mission
