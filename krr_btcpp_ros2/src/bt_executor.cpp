#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_ros2/bt_utils.hpp>
#include <vector>

#include "krr_btcpp_ros2/bt_executor.hpp"


void BTExecutor::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  std::vector<std::string> external_directories;

  // Get the values if they are set
  if (node()->get_parameter<std::vector<std::string>>("external_behavior_trees", external_directories))
  {

    for (auto directory_path : external_directories)
    {
      BT::LoadBehaviorTrees(factory, directory_path);
    }
  }
}


void BTExecutor::onTreeCreated(BT::GoalResources& session)
{
  RCLCPP_DEBUG(node()->get_logger(), "%s: onTreeCreated: tree %s created successfully.",
      node()->get_name(), session.tree.rootNode()->name().c_str());

  globalBlackboard(session)->set("payload", goalPayload(session));
  globalBlackboard(session)->set("prolog_query", node()->get_parameter("prolog_query_service").as_string());
  globalBlackboard(session)->set("prolog_assert", node()->get_parameter("prolog_assert_topic").as_string());
  globalBlackboard(session)->set("prolog_retract", node()->get_parameter("prolog_retract_topic").as_string());
  globalBlackboard(session)->createEntry("return_message", BT::TypeInfo::Create<std::string>());
}


bool BTExecutor::onGoalReceived(const std::string& tree_name, const std::string& payload)
{
  RCLCPP_DEBUG(node()->get_logger(), "%s: onGoalRecieved: {tree_name: '%s'; payload: '%s'.",
      node()->get_name(), tree_name.c_str(), payload.c_str());
  return true;
}

std::optional<std::string> BTExecutor::onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled, BT::GoalResources& session)
{
  RCLCPP_DEBUG(node()->get_logger(), 
      "%s: onTreeExecutionCompleted: {status: '%s'; was_cancelled: '%s'}",
      node()->get_name(), toStr(status), was_cancelled ? "true" : "false");

  std::string message;
  if (status == BT::NodeStatus::SUCCESS && !was_cancelled)
  {
    try
    {
      if (globalBlackboard(session)->get("return_message", message))
      {
        return message;
      } else {
        RCLCPP_DEBUG(node()->get_logger(), 
            "%s: onTreeExecutionCompleted: No return message from global blackboard",
            node()->get_name());
      }
    } 
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(node()->get_logger(), 
          "%s: onTreeExecutionCompleted: %s",
          node()->get_name(), e.what());
    }
  }
  return std::nullopt;
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<BTExecutor>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();
}
