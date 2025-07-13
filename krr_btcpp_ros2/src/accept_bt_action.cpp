#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/accept_bt_action.hpp"


bool AcceptAction::setGoal(Goal& goal)
{
  decision_msgs::msg::AlternativeArray chosen;

  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::Evaluation>("evaluation", goal.choice.evaluation))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "evaluation");
    return false;
  }
  if (!getInput<decision_msgs::msg::AlternativeArray>("choice", chosen))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "choice");
    return false;
  }

  n_chosen_ = goal.choice.chosen.size();
  goal.choice.chosen = chosen.alternatives;

  return true;
}


BT::NodeStatus AcceptAction::onResultReceived(const WrappedResult& wr)
{
  setOutput<bool>("accepted", wr.result->accepted);
  return wr.result->accepted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


BT::NodeStatus AcceptAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: goal is %.2f%% (%d/%u alternatives) completed",
      name().c_str(), 
      (float)feedback->num_accepted / (float)n_chosen_ * 100.0,
      feedback->num_accepted, 
      n_chosen_);
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AcceptAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void AcceptAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(AcceptAction, "Accept");
