#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/get_alternatives_bt_action.hpp"


bool GetAlternativesAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::Gap>("gap", goal.gap))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "gap");
    return false;

  }
  if (!getInput<int16_t>("max_matches", max_results_))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "max_matches");
    return false;
  }

  goal.maxresult = max_results_;

  return true;
}


BT::NodeStatus GetAlternativesAction::onResultReceived(const WrappedResult& wr)
{
  setOutput<decision_msgs::msg::AlternativeArray>("alternatives", wr.result->alternatives);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus GetAlternativesAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: %d unique alternatives matched (max %d)",
      name().c_str(), 
      feedback->num_matched,
      max_results_);
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus GetAlternativesAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void GetAlternativesAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(GetAlternativesAction, "GetAlternatives");
