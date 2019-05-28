cmake_minimum_required(VERSION 2.8.3)
project(selfie_control)

find_package(catkin REQUIRED COMPONENTS
  genmsg actionlib_msgs
  dynamic_reconfigure
  ackermann_msgs
  genmsg
  rospy
  roscpp
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/HeadingCoeff.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cmd_creator src/cmd_creator.cpp)
target_link_libraries(cmd_creator ${catkin_LIBRARIES})
add_dependencies(cmd_creator ${selfie_control_EXPORTED_TARGETS})
