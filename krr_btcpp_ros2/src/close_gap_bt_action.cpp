#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/gap_bt_actions.hpp"

#include <decision_msgs/msg/alternative_array.hpp>


bool CloseGapAction::setMessage(decision_msgs::msg::PrologClause& msg)
{
  std::string gap;
  decision_msgs::msg::AlternativeArray choice;

  // TODO: abstract away this call so that code isn't copied
  if (!getInput("gap", gap))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "gap");
    return false;
  }
  if (!getInput("choice", choice))
  {
    RCLCPP_ERROR(logger(), "%s: setRequest with error: no blackboard entry for {%s}", 
        name().c_str(), "choice");
    return false;
  }

  // guesstimate at how big it will need to be. This is unlikely to need to be resized more than once.
  msg.clause.resize((choice.alternatives.size() + 1) * 14); 
  msg.clause = "closed_with('" + gap + "', C) :- (";

  std::string prefix = "C = '";
  for (decision_msgs::msg::Alternative& a : choice.alternatives)
  {
    msg.clause.append(prefix + a.id);
    prefix = "'; C = '";
  }
  msg.clause.append("')");

  // TODO: for some reason this isn't writing to the output port.  
  // Maybe because this derrived from a ConditionNode?
  // setOutput("handle", "closed_with(" + gap + ", C)");
  return true;
}


// Register these nodes as plugins with the BT factory
CreateRosNodePlugin(CloseGapAction, "CloseGap");
