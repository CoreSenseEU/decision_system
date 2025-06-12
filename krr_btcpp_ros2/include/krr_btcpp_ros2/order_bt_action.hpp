#ifndef KRR_BT_CPP__ORDER_BT_ACTION_HPP_
#define KRR_BT_CPP__ORDER_BT_ACTION_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include <decision_msgs/msg/detail/weak_ordering__struct.hpp>
#include <string>

#include "decision_msgs/action/order.hpp"
#include "decision_msgs/msg/evaluation.hpp"
#include "decision_msgs/msg/weak_ordering.hpp"
#include "decision_msgs/msg/alternative_array.hpp"


class OrderAction : public BT::RosActionNode<decision_msgs::action::Order>
{
public:
  explicit OrderAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosActionNode<decision_msgs::action::Order>(name, config, params)
  {}

  /**
   * @brief Required method that creates list of BT ports
   *
   * Must call providedBasicPorts in it.
   *
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {    
    return providedBasicPorts({ 
        BT::InputPort<decision_msgs::msg::Evaluation>(
            "evaluation",
            "A matix of multi-dimensional values of each alternative."),
        BT::OutputPort<decision_msgs::msg::WeakOrdering>(
            "ordering",
            "A weak ordering of the choice set.")
        });
  }

  /** 
   * @brief Required callback that allows the user to set the goal message.
   *
   * @param goal The goal to be sent to the action server.
   *
   * @return false if the request should not be sent. In that case,
   * RosActionNode::onFailure(INVALID_GOAL) will be called.
   */
  bool setGoal(Goal& goal) override;

  /** 
   * @brief Required callback invoked when the result is received by the server.
   * 
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  /** 
   * @brief Optional callback invoked when the feedback is received.
   * 
   * It generally returns RUNNING, but the user can also use this callback to cancel the
   * current action and return SUCCESS or FAILURE.
   */
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  /**
   * @brief Optional callback invoked when something goes wrong. 
   *
   * It must return either SUCCESS or FAILURE.
   */
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  /**
   * @brief Optional callback executed when the node is halted. 
   *
   * Note that cancelGoal() is done automatically.
   */
  void onHalt() override;

private:
  // TODO: switch to a std::vector<decision_msgs::msg::Alternative> everywhere
  decision_msgs::msg::AlternativeArray alternatives_;

};

#endif
