cmake_minimum_required(VERSION 3.8)
project(decision_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Alternative.msg"
  "msg/AlternativeArray.msg"
  "msg/Assessment.msg"
  "msg/AssessmentMatrix.msg"
  "msg/Choice.msg"
  "msg/Cue.msg"
  "msg/CueArray.msg"
  "msg/CueWeights.msg"
  "msg/Decision.msg"
  "msg/Evaluation.msg"
  "msg/Feature.msg"
  "msg/Judgment.msg"
  "msg/OrderedEvaluation.msg"
  "msg/Preference.msg"
  "msg/PrologClause.msg"
  "msg/PrologAnswer.msg"
  "msg/PrologBinding.msg"
  "msg/WeakOrdering.msg"

  "srv/AssessAlternatives.srv"
  "srv/CreatePose.srv"
  "srv/PrologQuery.srv"

  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
