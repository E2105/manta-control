cmake_minimum_required(VERSION 2.8.3)
project(pwm_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  )

roslint_python()

catkin_package(
  CATKIN_DEPENDS
  rospy
  )
