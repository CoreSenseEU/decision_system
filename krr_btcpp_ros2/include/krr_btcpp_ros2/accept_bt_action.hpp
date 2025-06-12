#ifndef KRR_BT_CPP__ACCEPT_BT_ACTION_HPP_
#define KRR_BT_CPP__ACCEPT_BT_ACTION_HPP_

#include <behaviortree_ros2/bt_action_node.hpp>
#include <string>

// Required for BehaviorTree.CPP to convert strings in the XML behavior trees
// into ROS interface types
// TODO: Possibly this is because the "AssessAction" node uses a different definition,
// so it has already overwritten the standard one or something?
#include "krr_btcpp_ros2/convert_from_string.hpp"

#include "decision_msgs/action/accept.hpp"
#include "decision_msgs/msg/evaluation.hpp"
#include "decision_msgs/msg/alternative_array.hpp"


class AcceptAction : public BT::RosActionNode<decision_msgs::action::Accept>
{
public:
  explicit AcceptAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosActionNode<decision_msgs::action::Accept>(name, config, params)
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
        // This is optional as it is only used by some ACCEPT actions.
        // TODO: should we use two different kinds then?
        BT::InputPort<decision_msgs::msg::Evaluation>(
            "evaluation",
            "A matix of multi-dimensional values of each alternative."),
        BT::InputPort<decision_msgs::msg::AlternativeArray>(
            "choice",
            "The chosen set of alternatives."),
        BT::OutputPort<bool>(
            "accepted",
            "A status indicating if the decision was accepted (True) or not (False)."),
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
  uint16_t n_chosen_;

};

#endif
