cmake_minimum_required(VERSION 2.8.3)
project(sim)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build dmc and hydroX
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(dmc src/dmc.cpp)
target_link_libraries(dmc ${catkin_LIBRARIES})
add_dependencies(dmc sim_generate_messages_cpp)

add_executable(hydrometer src/hydrometer.cpp)
target_link_libraries(hydrometer ${catkin_LIBRARIES})
add_dependencies(hydrometer sim_generate_messages_cpp)