#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/plugins.hpp>
#include <rclcpp_action/client.hpp>

#include "krr_btcpp_ros2/assess_bt_action.hpp"

#include "decision_msgs/msg/assessment_matrix.hpp"


bool AssessAction::setGoal(Goal& goal)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput<decision_msgs::msg::CueArray>("cues", this->cues_))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "cues");
    return false;

  }
  if (!getInput<decision_msgs::msg::AlternativeArray>("alternatives", this->alternatives_))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "alternatives");
    return false;
  }

  goal.cues = this->cues_.cues;
  goal.alternatives = this->alternatives_.alternatives;

  return true;
}


BT::NodeStatus AssessAction::onResultReceived(const WrappedResult& wr)
{
  decision_msgs::msg::AssessmentMatrix matrix;
  matrix.cues= cues_.cues;
  matrix.alternatives = alternatives_.alternatives;
  matrix.scores = wr.result->scores;

  setOutput<decision_msgs::msg::AssessmentMatrix>("assessments", matrix);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus AssessAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  RCLCPP_INFO(logger(), "%s: onFeedback: goal is %.2f%% (%d/%lu) completed",
      name().c_str(), 
      (float)feedback->num_assessed / (float)cues_.cues.size() * 100.0,
      feedback->num_assessed, 
      cues_.cues.size());
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus AssessAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}


void AssessAction::onHalt()
{
  RCLCPP_WARN(logger(), "%s: onHalt with warning: goal was cancelled", name().c_str());
}

// Register this node as a plugin with the BT factory
CreateRosNodePlugin(AssessAction, "Assess");
