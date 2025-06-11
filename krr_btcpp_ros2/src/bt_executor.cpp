#include <rclcpp/rclcpp.hpp>

#include "krr_btcpp_ros2/bt_executor.hpp"


void BTExecutor::onTreeCreated(BT::Tree& tree)
{
  RCLCPP_DEBUG(node()->get_logger(), "%s: onTreeCreated: tree %s created successfully.",
      node()->get_name(), tree.rootNode()->name().c_str());
}


bool BTExecutor::onGoalReceived(const std::string& tree_name, const std::string& payload)
{
  RCLCPP_DEBUG(node()->get_logger(), "%s: onGoalRecieved: {tree_name: '%s'; payload: '%s'.",
      node()->get_name(), tree_name.c_str(), payload.c_str());
  return true;
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
