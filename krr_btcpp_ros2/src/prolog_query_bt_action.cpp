#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/prolog_bt_actions.hpp"


bool PrologQueryAction::setRequest(Request::SharedPtr& request)
{
  // TODO: abstract away this call so that code isn't copied
  if (!getInput("query", request->query.clause))
  {
    RCLCPP_ERROR(logger(), "%s: setRequest with error: no blackboard entry for {%s}", 
        name().c_str(), "query");
    return false;
  }
  if (!getInput("maxresult", request->maxresult))
  {
    RCLCPP_ERROR(logger(), "%s: setRequest with error: no blackboard entry for {%s}", 
        name().c_str(), "maxresult");
    return false;
  }


  return true;
}


BT::NodeStatus PrologQueryAction::onResponseReceived(const Response::SharedPtr& response)
{
  setOutput<std::vector<decision_msgs::msg::PrologAnswer>>("answers", response->answers);
  setOutput<bool>("success", response->success);
  return BT::NodeStatus::SUCCESS;
}


// Register these nodes as plugins with the BT factory
CreateRosNodePlugin(PrologQueryAction, "PrologQuery");
