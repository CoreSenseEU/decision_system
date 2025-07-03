#ifndef KRR_BT_CPP__PROLOG_BT_ACTION_HPP_
#define KRR_BT_CPP__PROLOG_BT_ACTION_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>
#include <string>

// Required for BehaviorTree.CPP to convert strings in the XML behavior trees
// into ROS interface types
// #include "krr_btcpp_ros2/convert_from_string.hpp"

#include "decision_msgs/srv/prolog_query.hpp"
#include "decision_msgs/msg/prolog_answer.hpp"


class PrologQueryAction : public BT::RosServiceNode<decision_msgs::srv::PrologQuery>
{
public:
  explicit PrologQueryAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosServiceNode<decision_msgs::srv::PrologQuery>(name, config, params)
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
            "query",
            "A query to submit to the Prolog knowledge base, without the terminating '.'"),
        BT::InputPort<int8_t>(
            "maxresult",
            -1,
            "The maximum number of desired results, or -1 to collect all results."),
        BT::OutputPort<std::vector<decision_msgs::msg::PrologAnswer>>(
            "answers",
            "A list of answers from the Prolog knowledge base."),
        BT::OutputPort<bool>(
            "success",
            "True if a query with no bindings is successful, False otherwise."),
        });
  }

 /** setRequest is a callback that allows the user to set
   * the request message (ServiceT::Request).
   *
   * @param request  the request to be sent to the service provider.
   *
   * @return false if the request should not be sent. In that case,
   * RosServiceNode::onFailure(INVALID_REQUEST) will be called.
   */
  bool setRequest(Request::SharedPtr& request) override;

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

};



class PrologFetchFromGapAction : public BT::RosServiceNode<decision_msgs::srv::PrologQuery>
{
public:
  explicit PrologFetchFromGapAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosServiceNode<decision_msgs::srv::PrologQuery>(name, config, params)
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
            "query",
            "A query to submit to the Prolog knowledge base, without the terminating '.'"),
        BT::InputPort<int8_t>(
            "maxresult",
            -1,
            "The maximum number of desired results, or -1 to collect all results."),
        BT::InputPort(
            "gap",
            "A unique identifier for the decision actively being solved."),
        BT::OutputPort(
            "target",
            "Answers matching the binding to '_T', separated by ';'"),
        });
  }

 /** setRequest is a callback that allows the user to set
   * the request message (ServiceT::Request).
   *
   * @param request  the request to be sent to the service provider.
   *
   * @return false if the request should not be sent. In that case,
   * RosServiceNode::onFailure(INVALID_REQUEST) will be called.
   */
  bool setRequest(Request::SharedPtr& request) override;

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

private:
  std::string gap_;

};



class CheckExistenceAction : public BT::RosServiceNode<decision_msgs::srv::PrologQuery>
{
public:
  explicit CheckExistenceAction(const std::string& name, const BT::NodeConfig& config,
               const BT::RosNodeParams& params)
    : BT::RosServiceNode<decision_msgs::srv::PrologQuery>(name, config, params)
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
            "query",
            "A query to submit to the Prolog knowledge base, without the terminating '.'"),
        BT::InputPort(
            "gap",
            "A unique identifier for the decision actively being solved."),
        });
  }

 /** setRequest is a callback that allows the user to set
   * the request message (ServiceT::Request).
   *
   * @param request  the request to be sent to the service provider.
   *
   * @return false if the request should not be sent. In that case,
   * RosServiceNode::onFailure(INVALID_REQUEST) will be called.
   */
  bool setRequest(Request::SharedPtr& request) override;

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

private:
  std::string gap_;

};
#endif
