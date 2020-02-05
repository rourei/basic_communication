cmake_minimum_required(VERSION 2.8.3)
project(dynamic_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_tutorials
#  CATKIN_DEPENDS dynamix_reconfigure roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(server src/server.cpp)

target_link_libraries( 
  server
  ${catkin_LIBRARIES}
)

add_executable(client src/client.cpp)

target_link_libraries(
  client
  ${catkin_LIBRARIES}
)
add_dependencies(server ${PROJECT_NAME}_gencfg)
add_dependencies(client ${PROJECT_NAME}_gencfg)
