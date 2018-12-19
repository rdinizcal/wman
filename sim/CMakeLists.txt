cmake_minimum_required(VERSION 2.8.3)
project(sim)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare ROS messages
# add_message_files(FILES std_msgs)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build dmc and hydro
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hydrometer src/hydrometer.cpp)
target_link_libraries(hydrometer ${catkin_LIBRARIES})
add_dependencies(hydrometer generate_messages_cpp)