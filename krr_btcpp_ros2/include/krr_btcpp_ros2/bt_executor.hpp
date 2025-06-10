#ifndef KRR_BT_CPP__BT_EXECUTOR_HPP_
#define KRR_BT_CPP__BT_EXECUTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/tree_execution_server.hpp>
#include <string>
#include <optional>
#include <memory>


class BTExecutor : public BT::TreeExecutionServer
{
public:
  explicit BTExecutor(const rclcpp::NodeOptions& options)
    : TreeExecutionServer(std::make_unique<rclcpp::Node>("bt_executor", options))
  {}

protected:
  /**
   * @brief Optional callback invoked when a goal is received and before the
   * tree is created. If it returns false, the goal will be rejected.
   */
  bool onGoalReceived(const std::string& tree_name, const std::string& payload) override;

  /**
   * @brief Optional callback invoked after the tree is created.
   * It can be used, for instance, to initialize a logger or the global blackboard.
   *
   * @param tree The tree that was created
   */
  void onTreeCreated(BT::Tree& tree) override;

  /**
   * @brief Optional callback invoked after the plugins were registered into
   * the BT::BehaviorTreeFactory.
   * It can be used to register additional custom nodes manually.
   *
   * @param factory The factory to use to register nodes
   */
  // void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override;

  /**
   * @brief Optional method invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
   */
  // std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;

  /**
   * @brief Optional callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
   *
   * @return if not std::nullopt, the string will be sent as [return_message] to the Action Client.
   */
  // std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;

  /**
   * @brief onLoopFeedback is a callback invoked at each loop, after tree.tickOnce().
   * If it returns a valid string, it will be sent as feedback to the Action Client.
   *
   * If you don't want to return any feedback, return std::nullopt.
   */
  // std::optional<std::string> onLoopFeedback() override;

};

#endif
