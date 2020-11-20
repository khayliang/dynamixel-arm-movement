cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_positions)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  ChangeTorque.srv
  GetPos.srv
  MoveToPos.srv
  SaveCurrentPos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(PROGRAMS
  scripts/position_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
