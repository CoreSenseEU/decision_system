cmake_minimum_required(VERSION 3.8)
project(decision_interfaces)

find_package(rosidl_default_generators REQUIRED)
# find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AcceptChoice.srv"
  "srv/AcceptRelational.srv"
  "srv/AcceptSatisficing.srv"
  "srv/AcceptDominating.srv"

  "msg/Alternative.msg"
  "msg/Feature.msg"
  "msg/Judgement.msg"
  # DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
