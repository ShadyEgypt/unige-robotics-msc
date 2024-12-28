cmake_minimum_required(VERSION 3.0.2)
project(assignment2_p1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  RobotStatus.msg
)

add_service_files(
  FILES
  GetLastTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib actionlib_msgs geometry_msgs nav_msgs sensor_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/action_client_node.py
  scripts/last_target_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
