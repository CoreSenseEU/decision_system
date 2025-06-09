cmake_minimum_required(VERSION 3.8)
project(krr_btcpp_ros2)

################
### Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


################
### Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(decision_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

set(DEPENDENCIES
  behaviortree_ros2
  behaviortree_cpp
  btcpp_ros2_interfaces
  std_msgs
  std_srvs
  nav2_msgs
  decision_msgs
)


################
### Add executables

# AssessAlternatives action BT node (Plugin version)
add_library(assess_plugin SHARED src/assess_action.cpp)
target_include_directories(assess_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(assess_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(assess_plugin ${DEPENDENCIES})

# add_executable(action_assess src/assess_action.cpp)
# target_include_directories(action_assess PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(action_assess ${DEPENDENCIES})

# Behavior Tree execution server
add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_executor ${DEPENDENCIES})

#
# # Navigate action BT Node 
# add_executable(action_navigate src/action_navigate.cpp)
# ament_target_dependencies(action_navigate ${DEPENDENCIES})
#
# # Pick action BT Node 
# add_executable(action_pick src/action_pick.cpp)
# ament_target_dependencies(action_pick ${DEPENDENCIES})
#
# # Pick action BT Node 
# add_executable(action_place src/action_place.cpp)
# ament_target_dependencies(action_place ${DEPENDENCIES})


###############
### Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


###############
### Install targets and directories
install(TARGETS
  assess_plugin
  # action_assess
  bt_executor
  # action_navigate
  # action_pick
  # action_place
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install plugins for other packages to load
install(TARGETS
  assess_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

# Install headers for plugins
install(DIRECTORY
  include/
  DESTINATION include
)

# Export plugin dependencies for downstream packages
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(
  include
)

ament_package()
