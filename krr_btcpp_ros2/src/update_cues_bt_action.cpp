#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/update_cues_bt_action.hpp"


bool UpdateCuesAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput("gap", goal.gap))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "gap");
    return false;

  }
  if (!getInput<decision_msgs::msg::CueArray>("cues", goal.previous_cues))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "cues");
    return false;
  }

  return true;
}


BT::NodeStatus UpdateCuesAction::onResultReceived(const WrappedResult& wr)
{
  setOutput<decision_msgs::msg::CueArray>("cues", wr.result->cues);
  return BT::NodeStatus::SUCCESS;
}


// BT::NodeStatus UpdateCuesAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
// {
//   return BT::NodeStatus::RUNNING;
// }


BT::NodeStatus UpdateCuesAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void UpdateCuesAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(UpdateCuesAction, "UpdateCues");
