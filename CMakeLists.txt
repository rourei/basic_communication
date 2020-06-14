cmake_minimum_required(VERSION 2.8.3)
project(basic_communication)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  rospy_message_converter
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

add_message_files(
  FILES
  Kreditkarte.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# ---------------------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------------------
add_executable(dyn_rec_server src/dyn_rec_server.cpp)

target_link_libraries( 
  dyn_rec_server
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(dyn_rec_client src/dyn_rec_client.cpp)

target_link_libraries(
  dyn_rec_client
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(dyn_rec_client_double src/dyn_rec_client_double.cpp)

target_link_libraries(
  dyn_rec_client_double
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(latched_talker src/latched_talker.cpp)

target_link_libraries(
  latched_talker
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(latched_listener src/latched_listener.cpp)

target_link_libraries(
  latched_listener
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(param_setter src/param_setter.cpp)

target_link_libraries(
  param_setter
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(param_cached_getter src/param_cached_getter.cpp)

target_link_libraries(
  param_cached_getter
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_executable(regex_string_splitting src/regex_string_splitting.cpp)

target_link_libraries(
  regex_string_splitting
  ${catkin_LIBRARIES}
)

# ---------------------------------------------------------------------------------------

add_executable(serial_interface_test
	src/serial_interface_test.cpp
)

target_link_libraries(serial_interface_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# ---------------------------------------------------------------------------------------
add_dependencies(dyn_rec_server ${PROJECT_NAME}_gencfg)
add_dependencies(dyn_rec_client ${PROJECT_NAME}_gencfg)
add_dependencies(dyn_rec_client_double ${PROJECT_NAME}_gencfg)
