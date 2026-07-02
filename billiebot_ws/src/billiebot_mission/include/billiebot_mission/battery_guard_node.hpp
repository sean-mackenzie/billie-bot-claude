#ifndef BILLIEBOT_MISSION__BATTERY_GUARD_NODE_HPP_
#define BILLIEBOT_MISSION__BATTERY_GUARD_NODE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace billiebot_mission
{

/**
 * Battery guard condition node.
 * Returns FAILURE if battery voltage is below threshold (triggers SAFE mode).
 */
class BatteryGuardNode : public BT::ConditionNode
{
public:
  BatteryGuardNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace billiebot_mission

#endif  // BILLIEBOT_MISSION__BATTERY_GUARD_NODE_HPP_
