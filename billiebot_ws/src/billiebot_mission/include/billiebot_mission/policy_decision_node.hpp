#ifndef BILLIEBOT_MISSION__POLICY_DECISION_NODE_HPP_
#define BILLIEBOT_MISSION__POLICY_DECISION_NODE_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace billiebot_mission
{

/**
 * PolicyDecision BT node — decides what action to take given context.
 *
 * MVP: ObserveOnlyPolicy (always returns OBSERVE).
 * Future: pluginlib-based policy selection for Behavior AI.
 *
 * Output ports:
 *   - "decision": one of {OBSERVE, APPROACH, RETREAT, SPEAK, TREAT}
 */
class PolicyDecisionNode : public BT::SyncActionNode
{
public:
  PolicyDecisionNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string policy_name_;
};

}  // namespace billiebot_mission

#endif  // BILLIEBOT_MISSION__POLICY_DECISION_NODE_HPP_
