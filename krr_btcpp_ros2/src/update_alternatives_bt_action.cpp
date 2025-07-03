#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/update_alternatives_bt_action.hpp"


bool UpdateAlternativesAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::Gap>("gap", goal.gap))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "gap");
    return false;

  }
  if (!getInput<decision_msgs::msg::AlternativeArray>("previous_choice", goal.previous_taken))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "previous_choice");
    return false;
  }
  if (!getInput<decision_msgs::msg::AlternativeArray>("choice_set", goal.previous_choice_set))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "choice_set");
    return false;
  }

  return true;
}


BT::NodeStatus UpdateAlternativesAction::onResultReceived(const WrappedResult& wr)
{
  setOutput<decision_msgs::msg::AlternativeArray>("choice_set", wr.result->choice_set);
  return BT::NodeStatus::SUCCESS;
}


// BT::NodeStatus UpdateAlternativesAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
// {
//   return BT::NodeStatus::RUNNING;
// }


BT::NodeStatus UpdateAlternativesAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void UpdateAlternativesAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(UpdateAlternativesAction, "UpdateAlternatives");
