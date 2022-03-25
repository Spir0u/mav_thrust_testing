cmake_minimum_required(VERSION 3.0.2)
project(thrust_test_controller_geranos)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
  cfg/thrust_test.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

