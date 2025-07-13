#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/adapt_decision_components_bt_action.hpp"


bool AdaptDecisionComponentsAction::setGoal(Goal& goal)
{
  if (!getInput("config", goal.params_file))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "config");
    return false;
  }

  return true;
}


BT::NodeStatus AdaptDecisionComponentsAction::onResultReceived(const WrappedResult& wr)
{
  if (!wr.result->success)
  {
    RCLCPP_ERROR(logger(), "%s: onResultRecieved failed with message: {%s}", 
        name().c_str(), wr.result->reason.c_str());
  }
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus AdaptDecisionComponentsAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: %u/%u components successfully adapted",
      name().c_str(), 
      feedback->num_adapted,
      feedback->num_to_adapt);
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AdaptDecisionComponentsAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void AdaptDecisionComponentsAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(AdaptDecisionComponentsAction, "AdaptDecisionComponents");
