cmake_minimum_required(VERSION 2.8.3)
project(sensor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  roslint
)

roslint_python(scripts/bno055_interface.py  scripts/ms5837_interface.py)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
