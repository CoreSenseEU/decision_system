#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/prolog_bt_actions.hpp"


bool CheckExistenceAction::setRequest(Request::SharedPtr& request)
{
  std::string query;

  // TODO: abstract away this call so that code isn't copied
  if (!getInput("gap", gap_))
  {
    RCLCPP_ERROR(logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        name().c_str(), "gap");
    return false;
  }
  if (!getInput("query", query))
  {
    RCLCPP_ERROR(logger(), "%s: setRequest with error: no blackboard entry for {%s}", 
        name().c_str(), "query");
    return false;
  }

  // Only need a proof, so one answer will do
  request->maxresult = 1;

  std::size_t next = 0;
  std::size_t last = 0;

  // guesstimate at how big it will need to be. This is unlikely to need to be resized more than once.
  request->query.clause.resize(query.size() + 4 * gap_.size()); 
  request->query.clause = "gap(" + gap_ +"), ";

  // Replace any occurance of the sequence "_G" in the query with the gap
  while ((next = query.find("_G", last)) != std::string::npos) {
    request->query.clause.append(query.substr(last, next - last));
    request->query.clause.append(gap_);
    last = next + 2; // 2 is length of sequence
  }
  request->query.clause.append(query.substr(last));

  return true;
}


BT::NodeStatus CheckExistenceAction::onResponseReceived(const Response::SharedPtr& response)
{
  return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// Register these nodes as plugins with the BT factory
CreateRosNodePlugin(CheckExistenceAction, "CheckExistence");
