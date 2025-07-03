#include <behaviortree_ros2/plugins.hpp>

#include "krr_btcpp_ros2/prolog_bt_actions.hpp"

#include <decision_msgs/msg/prolog_binding.hpp>


bool PrologFetchFromGapAction::setRequest(Request::SharedPtr& request)
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
  if (!getInput("maxresult", request->maxresult))
  {
    RCLCPP_ERROR(logger(), "%s: setRequest with error: no blackboard entry for {%s}", 
        name().c_str(), "maxresult");
    return false;
  }

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


BT::NodeStatus PrologFetchFromGapAction::onResponseReceived(const Response::SharedPtr& response)
{
  std::string answer_str;
  std::string prefix = "";
  bool bound;

  // Fail if there were no answers
  // TODO: should this also fail if the answer was 'True', but there were no bindings?
  if (!response->success)
  {
    return BT::NodeStatus::FAILURE;
  }


  for (decision_msgs::msg::PrologAnswer answer : response->answers)
  {
    bound = false;
    for (decision_msgs::msg::PrologBinding binding : answer.bindings)
    {
      if (binding.key == "_T")
      {
        answer_str.append(prefix + binding.value);
        prefix = ";";
        bound = true;
        break;
      }
    }
  
    // No binding for _T.  This should not be reachable if all went well...
    if (!bound)
    {
      RCLCPP_ERROR(logger(), "%s: onResponseRecieved with error: no binding for _T in prolog answer", 
          name().c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  setOutput("target", answer_str);
  return BT::NodeStatus::SUCCESS;
}

// Register these nodes as plugins with the BT factory
CreateRosNodePlugin(PrologFetchFromGapAction, "PrologFetchFromGap");
