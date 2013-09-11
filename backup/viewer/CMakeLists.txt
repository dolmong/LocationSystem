cmake_minimum_required(VERSION 2.8.3)
project(viewer)

find_package(
	catkin REQUIRED
	COMPONENTS image_transport roscpp sensor_msgs cv_bridge genmsg
)
find_package(
	OpenCV REQUIRED
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES sensor_msgs
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
	LIBRARIES locationsystem
	CATKIN_DEPENDS image_transport roscpp sensor_msgs
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
	viewer_node
	src/viewer.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
	viewer_node
	viewer_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	viewer_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
