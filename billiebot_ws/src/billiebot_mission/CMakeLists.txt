cmake_minimum_required(VERSION 3.8)
project(billiebot_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(billiebot_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# C++ BT nodes
add_library(billiebot_bt_nodes SHARED
  src/policy_decision_node.cpp
  src/battery_guard_node.cpp
  src/estop_guard_node.cpp
)

target_include_directories(billiebot_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(billiebot_bt_nodes
  rclcpp
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  std_msgs
  billiebot_interfaces
  pluginlib
)

install(TARGETS billiebot_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Python action servers
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  billiebot_mission/approach_dog_server.py
  billiebot_mission/retreat_server.py
  billiebot_mission/speak_server.py
  billiebot_mission/dispense_treat_server.py
  billiebot_mission/mission_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Config and launch
install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
