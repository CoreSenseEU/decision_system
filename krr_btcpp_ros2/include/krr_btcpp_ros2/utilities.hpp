#ifndef KRR_BT_CPP__UTILITIES_HPP_
#define KRR_BT_CPP__UTILITIES_HPP_

#include <string>
#include "behaviortree_ros2/bt_action_node.hpp"

template<class T>
inline bool hasInput(std::string key, T& destination, BT::RosActionNode<T>* node)
{
  if (!node->template getInput<T>(key, destination))
  {
    RCLCPP_ERROR(node->logger(), "%s: setGoal with error: no blackboard entry for {%s}", 
        node->name().c_str(), key);
    return false;
  }
  return true;
}




#endif 
