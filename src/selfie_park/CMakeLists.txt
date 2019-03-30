cmake_minimum_required(VERSION 2.8.3)
project(selfie_park)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs nav_msgs ackermann_msgs tf selfie_msgs visualization_msgs)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES park.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(park_server src/park_server.cpp)

target_link_libraries(
  park_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  park_server
  ${selfie_park_EXPORTED_TARGETS}
)

add_executable(park_client src/park_client.cpp )

target_link_libraries(
  park_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  park_client
  ${selfie_park_EXPORTED_TARGETS}
)