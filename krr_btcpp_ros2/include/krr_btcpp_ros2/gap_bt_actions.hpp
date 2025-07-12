#ifndef KRR_BT_CPP__GAP_BT_ACTION_HPP_
#define KRR_BT_CPP__GAP_BT_ACTION_HPP_

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <string>

// Required for BehaviorTree.CPP to convert strings in the XML behavior trees
// into ROS interface types
// #include "krr_btcpp_ros2/convert_from_string.hpp"

// #include "decision_msgs/srv/prolog_query.hpp"
#include "decision_msgs/msg/prolog_clause.hpp"
#include "decision_msgs/msg/alternative_array.hpp"


class CloseGapAction : public BT::RosTopicPubNode<decision_msgs::msg::PrologClause>
{
public:
  explicit CloseGapAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosTopicPubNode<decision_msgs::msg::PrologClause>(name, config, params)
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
        BT::InputPort(
            "gap",
            "A unique identifier for the decision actively being solved."),
        BT::InputPort<decision_msgs::msg::AlternativeArray>(
            "choice",
            "The chosen set of alternatives."),
        // TODO: for some reason this isn't writing to the output port.  
        // Maybe because this derrived from a ConditionNode?
        // BT::OutputPort<std::string>(
        //     "handle",
        //     "A Prolog query to retrieve all the chosen alternatives."),
        });
  }

  /**
   * @brief setMessage is a callback invoked in tick to allow the user to pass
   * the message to be published.
   *
   * @param msg the message.
   * @return  return false if anything is wrong and we must not send the message.
   * the Condition will return FAILURE.
   */
  bool setMessage(decision_msgs::msg::PrologClause& msg) override;

protected:
  rclcpp::Logger logger()
  {
    return node_->get_logger();
  }

};

// class CopyGapAction : public BT::RosServiceNode<decision_msgs::srv::PrologQuery>
// {
// public:
//   explicit CopyGapAction(const std::string& name, const BT::NodeConfig& config,
//                const BT::RosNodeParams& params)
//     : BT::RosServiceNode<decision_msgs::srv::PrologQuery>(name, config, params)
//   {}
//
//   /**
//    * @brief Required method that creates list of BT ports
//    *
//    * Must call providedBasicPorts in it.
//    *
//    * @return PortsList Containing basic ports along with node-specific ports
//    */
//   static BT::PortsList providedPorts()
//   {    
//     return providedBasicPorts({ 
//         BT::InputPort(
//             "parent",
//             "The id of a gap to copy."),
//         BT::OutputPort(
//             "child",
//             "The new gap id."),
//         });
//   }
//
//  /** setRequest is a callback that allows the user to set
//    * the request message (ServiceT::Request).
//    *
//    * @param request  the request to be sent to the service provider.
//    *
//    * @return false if the request should not be sent. In that case,
//    * RosServiceNode::onFailure(INVALID_REQUEST) will be called.
//    */
//   bool setRequest(Request::SharedPtr& request) override;
//
//   /** Callback invoked when the response is received by the server.
//    * It is up to the user to define if this returns SUCCESS or FAILURE.
//    */
//   BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
//
// };

#endif
