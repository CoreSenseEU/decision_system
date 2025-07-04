#ifndef KRR_BT_CPP__CONVERT_FROM_STRING_HPP_
#define KRR_BT_CPP__CONVERT_FROM_STRING_HPP_

#include "behaviortree_cpp/basic_types.h"

#include "decision_msgs/msg/cue.hpp"
#include "decision_msgs/msg/cue_array.hpp"
#include "decision_msgs/msg/alternative.hpp"
#include "decision_msgs/msg/alternative_array.hpp"

// #include "krr_btcpp_ros2/utilities.hpp"


// String conversion functions for ROS message types
// TODO: implement convertFromString() for ROS YAML messages with dynmsg
// https://github.com/osrf/dynamic_message_introspection/tree/main
namespace BT
{
  /**
   * @brief Convert from a string to a ROS decision_msgs.msg.Cue.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated ports
   * of type decision_msgs::msg::Cue.
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::Cue whose ID was contained in the string.
   */
  template <> inline decision_msgs::msg::Cue convertFromString(StringView str)
  {
    decision_msgs::msg::Cue cue;
    cue.id = str;
    return cue;
  }

  /**
   * @brief Convert from a string to a ROS decision_msgs.msg.Alternative.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated ports
   * of type decision_msgs::msg::Alternative.
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::Alternative whose ID was contained in the string.
   */
  template <> inline decision_msgs::msg::Alternative convertFromString(StringView str)
  {
    decision_msgs::msg::Alternative alternative;
    alternative.id = str;
    return alternative;
  }

  /**
   * @brief Convert from a string to a ROS decision_msgs.msg.CueArray.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated
   * ports of type decision_msgs::msg::CueArray. 
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::CueArray of Cues whose IDs are assumed to be
   * strings separated by the character ';' 
   * e.g. "blue;green;#06B4C1;yellow"
   */
  template <> inline decision_msgs::msg::CueArray convertFromString(StringView str)
  {
    decision_msgs::msg::CueArray msg;

    if (str.size() == 0)
    {
      return msg;
    }

    // Take in strings separated by the character ';'
    // e.g. "blue;green;#06B4C1;yellow"
    std::vector<std::string> entries = convertFromString<std::vector<std::string>>(str);
    msg.cues.reserve(entries.size());
    for (std::string entry : entries) {
      msg.cues.emplace_back(convertFromString<decision_msgs::msg::Cue>(entry));
    }
    return msg;
  }

  /**
   * @brief Convert from a string to a ROS decision_msgs.msg.AlternativeArray.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated
   * ports of type decision_msgs::msg::AlternativeArray. 
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::AlternativeArray of Alternatives whose IDs
   * are assumed to be strings separated by the character ';' 
   * e.g. "blue;green;#06B4C1;yellow"
   */
  template <> inline decision_msgs::msg::AlternativeArray convertFromString(StringView str)
  {
    decision_msgs::msg::AlternativeArray msg;

    if (str.size() == 0)
    {
      return msg;
    }

    // Take in strings separated by the character ';'
    // e.g. "blue;green;#06B4C1;yellow"
    std::vector<std::string> entries = convertFromString<std::vector<std::string>>(str);
    msg.alternatives.reserve(entries.size());
    for (std::string entry : entries) {
      msg.alternatives.emplace_back(convertFromString<decision_msgs::msg::Alternative>(entry));
    }
    return msg;
  }
} // namespace BT

#endif
