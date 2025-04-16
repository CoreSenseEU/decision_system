cmake_minimum_required(VERSION 3.8)
project(decision_interfaces)

find_package(rosidl_default_generators REQUIRED)
# find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AcceptChoice.srv"
  "srv/AcceptRelational.srv"
  "srv/AcceptSatisficing.srv"
  "srv/AcceptDominating.srv"
  "srv/TakeN.srv"
  "srv/TakeRelative.srv"

  "msg/Alternative.msg"
  "msg/Feature.msg"
  "msg/Judgment.msg"
  "msg/Choice.msg"
  "msg/WeakOrdering.msg"
  # DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
