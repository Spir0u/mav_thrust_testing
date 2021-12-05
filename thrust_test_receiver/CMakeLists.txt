cmake_minimum_required(VERSION 3.0.2)
project(thrust_test_receiver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
#   message_generation
#   mavros_msgs
)

# add_message_files(FILES
#   ESCTelemetryItem.msg
# )
# generate_messages(DEPENDENCIES std_msgs mavros_msgs)

catkin_package(
  # CATKIN_DEPENDS message_runtime 
)

# add_executable(thrust_test_receiver_node.py src/thrust_test_receiver_node.py)
# add_dependencies(thrust_test_receiver_node.py ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

