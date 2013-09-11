cmake_minimum_required(VERSION 2.8.3)
project(matching_test)

find_package(
	catkin REQUIRED
	COMPONENTS cv_bridge image_transport roscpp sensor_msgs gencpp
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES matching_test
	CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
	DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
	matching_test_node
	src/filesystem.cpp
	src/improc.cpp
	src/surf.cpp
	src/viewer.cpp
	src/matching.cpp
	src/main.cpp
)

add_dependencies(
	matching_test_node
	matching_test_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	matching_test_node
	${catkin_LIBRARIES}
)
