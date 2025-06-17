#ifndef KRR_BT_CPP__CONVERT_FROM_STRING_HPP_
#define KRR_BT_CPP__CONVERT_FROM_STRING_HPP_

#include <algorithm>
#include <ostream>
#include "behaviortree_cpp/basic_types.h"

#include "decision_msgs/msg/cue.hpp"
#include "decision_msgs/msg/cue_array.hpp"
#include "decision_msgs/msg/alternative.hpp"
#include "decision_msgs/msg/alternative_array.hpp"
#include "decision_msgs/msg/cs_template.hpp"
#include "decision_msgs/msg/gap.hpp"

// #include "krr_btcpp_ros2/utilities.hpp"


// String conversion functions for ROS message types
// TODO: implement convertFromString() for ROS YAML messages with
// [LibYAML](https://pyyaml.org/wiki/LibYAML)
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
   * @brief Convert from a string to a ROS decision_msgs.msg.CSTemplate.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated ports
   * of type decision_msgs::msg::CSTemplate.
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::CSTemplate with a prolog query contained in the string.
   */
  template <> inline decision_msgs::msg::CSTemplate convertFromString(StringView str)
  {
    decision_msgs::msg::CSTemplate template_;
    template_.query = str;
    return template_;
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

    // Take in strings separated by the character ';'
    // e.g. "blue;green;#06B4C1;yellow"
    std::vector<std::string> entries = convertFromString<std::vector<std::string>>(str);
    msg.alternatives.reserve(entries.size());
    for (std::string entry : entries) {
      msg.alternatives.emplace_back(convertFromString<decision_msgs::msg::Alternative>(entry));
    }
    return msg;
  }

  /**
   * @brief Convert from a string to a ROS decision_msgs.msg.Gap.
   *
   * This is used by BehaviorTree.CPP to parse XML behavior trees for templated
   * ports of type decision_msgs::msg::Gap. 
   *
   * @param BT::StringView str The string to convert from.
   *
   * @return a decision_msgs::msg::Gap of CSTemplates whose queries
   * are assumed to be strings separated by the sequence "\;" 
   * 
   * WARNING: there are many parsing edge cases to break this, ignored for now.
   * Please carefully review any prolog queries.  "\;" is an illegal prolog escape character so collisions may be invalid anyway.
   *
   * TODO: parse carefully and check for those ^
   *
   * e.g. "color(A) ; flavor(A)\;description(A)"
   *    --> two templates with queries: "color(A) ; flavor(A)" and "description(A)"
   */
  template <> inline decision_msgs::msg::Gap convertFromString(StringView str)
  {
    decision_msgs::msg::Gap gap;

    std::size_t next = 0;
    std::size_t last = 0;

    // Take in strings separated by the sequence "\;"
    // adapted from https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c#comment44856986_14266139
    while ((next = str.find("\\;", last)) != std::string::npos) {
      gap.templates.emplace_back(
          convertFromString<decision_msgs::msg::CSTemplate>(str.substr(last, next - last)));
      last = next + 2; // 2 is length of sequence
    }
    gap.templates.emplace_back(
        convertFromString<decision_msgs::msg::CSTemplate>(str.substr(last)));

    return gap;
  }
} // namespace BT

#endif
