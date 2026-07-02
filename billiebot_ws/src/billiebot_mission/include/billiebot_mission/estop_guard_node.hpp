#ifndef BILLIEBOT_MISSION__ESTOP_GUARD_NODE_HPP_
#define BILLIEBOT_MISSION__ESTOP_GUARD_NODE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace billiebot_mission
{

/**
 * E-stop guard condition node.
 * Returns FAILURE if e-stop is engaged (triggers SAFE mode).
 */
class EStopGuardNode : public BT::ConditionNode
{
public:
  EStopGuardNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace billiebot_mission

#endif  // BILLIEBOT_MISSION__ESTOP_GUARD_NODE_HPP_
