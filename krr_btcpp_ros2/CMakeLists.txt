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
### Add executables and plugins

# Behavior Tree execution server
add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bt_executor ${DEPENDENCIES})

# GetAlternatives BT::ActionNode (Plugin version)
add_library(get_alternatives_plugin SHARED src/get_alternatives_bt_action.cpp)
target_include_directories(get_alternatives_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(get_alternatives_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(get_alternatives_plugin ${DEPENDENCIES})

# Assess BT::ActionNode (Plugin version)
add_library(assess_plugin SHARED src/assess_bt_action.cpp)
target_include_directories(assess_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(assess_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(assess_plugin ${DEPENDENCIES})

# Aggregate BT::ActionNode (Plugin version)
add_library(aggregate_plugin SHARED src/aggregate_bt_action.cpp)
target_include_directories(aggregate_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(aggregate_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(aggregate_plugin ${DEPENDENCIES})

# Order BT::ActionNode (Plugin version)
add_library(order_plugin SHARED src/order_bt_action.cpp)
target_include_directories(order_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(order_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(order_plugin ${DEPENDENCIES})

# Take BT::ActionNode (Plugin version)
add_library(take_plugin SHARED src/take_bt_action.cpp)
target_include_directories(take_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(take_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(take_plugin ${DEPENDENCIES})

# Accept BT::ActionNode (Plugin version)
add_library(accept_plugin SHARED src/accept_bt_action.cpp)
target_include_directories(accept_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(accept_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(accept_plugin ${DEPENDENCIES})

# TreeExecutionClient BT::ActionNode (Plugin version)
add_library(tree_execution_client_plugin SHARED src/tree_execution_client_bt_action.cpp)
target_include_directories(tree_execution_client_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(tree_execution_client_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(tree_execution_client_plugin ${DEPENDENCIES})

# AssembleDecisionHeuristic BT::ActionNode (Plugin version)
add_library(assemble_decision_heuristic_plugin SHARED src/assemble_decision_heuristic_bt_action.cpp)
target_include_directories(assemble_decision_heuristic_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(assemble_decision_heuristic_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(assemble_decision_heuristic_plugin ${DEPENDENCIES})

# AssembleDecisionHeuristic BT::ActionNode (Plugin version)
add_library(adapt_decision_components_plugin SHARED src/adapt_decision_components_bt_action.cpp)
target_include_directories(adapt_decision_components_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(adapt_decision_components_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(adapt_decision_components_plugin ${DEPENDENCIES})

# PrologQuery, BT::ActionNode (Plugin version)
add_library(prolog_query_plugin SHARED src/prolog_query_bt_action.cpp)
target_include_directories(prolog_query_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(prolog_query_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(prolog_query_plugin ${DEPENDENCIES})

# PrologFetchFromGap, BT::ActionNode (Plugin version)
add_library(prolog_fetch_from_gap_plugin SHARED src/prolog_fetch_from_gap_bt_action.cpp)
target_include_directories(prolog_fetch_from_gap_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(prolog_fetch_from_gap_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(prolog_fetch_from_gap_plugin ${DEPENDENCIES})

# CheckExistence, BT::ActionNode (Plugin version)
add_library(check_existence_plugin SHARED src/check_existence_bt_action.cpp)
target_include_directories(check_existence_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(check_existence_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(check_existence_plugin ${DEPENDENCIES})

# CloseGap, BT::ActionNode (Plugin version)
add_library(close_gap_plugin SHARED src/close_gap_bt_action.cpp)
target_include_directories(close_gap_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(close_gap_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(close_gap_plugin ${DEPENDENCIES})

# UpdateAlternatives, BT::ActionNode (Plugin version)
add_library(update_alternatives_plugin SHARED src/update_alternatives_bt_action.cpp)
target_include_directories(update_alternatives_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(update_alternatives_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(update_alternatives_plugin ${DEPENDENCIES})

# UpdateCues, BT::ActionNode (Plugin version)
add_library(update_cues_plugin SHARED src/update_cues_bt_action.cpp)
target_include_directories(update_cues_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(update_cues_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(update_cues_plugin ${DEPENDENCIES})





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
  bt_executor
  # get_alternatives_plugin
  assess_plugin
  aggregate_plugin
  order_plugin
  take_plugin
  accept_plugin
  tree_execution_client_plugin
  assemble_decision_heuristic_plugin
  adapt_decision_components_plugin
  prolog_query_plugin
  prolog_fetch_from_gap_plugin
  check_existence_plugin
  close_gap_plugin
  update_alternatives_plugin
  update_cues_plugin
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
  # get_alternatives_plugin
  assess_plugin
  aggregate_plugin
  order_plugin
  take_plugin
  accept_plugin
  tree_execution_client_plugin
  assemble_decision_heuristic_plugin
  adapt_decision_components_plugin
  prolog_query_plugin
  prolog_fetch_from_gap_plugin
  check_existence_plugin
  close_gap_plugin
  update_alternatives_plugin
  update_cues_plugin
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
