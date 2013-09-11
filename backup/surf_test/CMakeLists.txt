cmake_minimum_required(VERSION 2.8.3)
project(surf_test)

find_package(
	catkin REQUIRED
	COMPONENTS cv_bridge image_transport roscpp sensor_msgs gencpp
)
find_package(
	OpenCV REQUIRED
)

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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES surf_test
	CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
	DEPENDS OpenCV system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
	surf_test_node
	src/surf/surf.cpp
	src/filesystem/filesystem.cpp
	src/viewer/viewer.cpp
	src/improc/improc.cpp
	src/main.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
	surf_test_node
	surf_test_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	surf_test_node
	${catkin_LIBRARIES}
	${OPENCV_LIBRARIES}
)
