#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/aggregate_bt_action.hpp"


bool AggregateAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::AssessmentMatrix>("assessments", goal.assessments))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "assessments");
    return false;

  }

  alternatives_.alternatives = goal.assessments.alternatives;

  return true;
}


BT::NodeStatus AggregateAction::onResultReceived(const WrappedResult& wr)
{
  decision_msgs::msg::Evaluation evaluation;
  evaluation.alternatives = alternatives_.alternatives;
  evaluation.axes = wr.result->axes;
  evaluation.scores = wr.result->scores;

  setOutput<decision_msgs::msg::Evaluation>("evaluation", evaluation);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus AggregateAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: goal is %.2f%% (%d/%lu alternatives) completed",
      name().c_str(), 
      (float)feedback->num_judged / (float)alternatives_.alternatives.size() * 100.0,
      feedback->num_judged, 
      alternatives_.alternatives.size());
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AggregateAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void AggregateAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(AggregateAction, "Aggregate");
