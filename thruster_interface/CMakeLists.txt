cmake_minimum_required(VERSION 2.8.3)
project(thruster_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  roslint
)

add_service_files(FILES
    ThrustersEnable.srv)

roslint_python()

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  message_runtime
)
