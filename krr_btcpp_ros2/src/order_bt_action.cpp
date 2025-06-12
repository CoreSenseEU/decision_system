#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/order_bt_action.hpp"


bool OrderAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::Evaluation>("evaluation", goal.evaluation))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "evaluation");
    return false;

  }

  alternatives_.alternatives = goal.evaluation.alternatives;

  return true;
}


BT::NodeStatus OrderAction::onResultReceived(const WrappedResult& wr)
{
  decision_msgs::msg::WeakOrdering ordering;
  ordering.alternatives = alternatives_.alternatives;
  ordering.ranks = wr.result->ranks;

  setOutput<decision_msgs::msg::WeakOrdering>("ordering", ordering);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus OrderAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: goal is %.2f%% (%d/%lu ranks) completed",
      name().c_str(), 
      (float)feedback->num_ordered / (float)alternatives_.alternatives.size() * 100.0,
      feedback->num_ordered, 
      alternatives_.alternatives.size());
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus OrderAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void OrderAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(OrderAction, "Order");
