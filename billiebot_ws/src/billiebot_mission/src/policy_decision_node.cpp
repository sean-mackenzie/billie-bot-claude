#include "billiebot_mission/policy_decision_node.hpp"

namespace billiebot_mission
{

PolicyDecisionNode::PolicyDecisionNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  policy_name_("ObserveOnlyPolicy")
{
  // Read policy from input port if available
  auto policy_input = getInput<std::string>("policy");
  if (policy_input) {
    policy_name_ = policy_input.value();
  }
}

BT::PortsList PolicyDecisionNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("policy", "ObserveOnlyPolicy",
                                "Policy plugin name"),
    BT::InputPort<std::string>("dog_state", "",
                                "Current dog state"),
    BT::InputPort<double>("stress_proxy", 0.0,
                           "Current stress proxy value"),
    BT::OutputPort<std::string>("decision", "Action decision"),
  };
}

BT::NodeStatus PolicyDecisionNode::tick()
{
  // MVP: ObserveOnlyPolicy always returns OBSERVE
  if (policy_name_ == "ObserveOnlyPolicy") {
    setOutput("decision", std::string("OBSERVE"));
    return BT::NodeStatus::SUCCESS;
  }

  // Future: load policy via pluginlib and call decide(context)
  // For now, default to OBSERVE
  setOutput("decision", std::string("OBSERVE"));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace billiebot_mission
