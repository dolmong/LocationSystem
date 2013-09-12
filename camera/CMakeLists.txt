cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(
	catkin REQUIRED
	COMPONENTS cv_bridge image_transport roscpp sensor_msgs std_msgs
)
find_package(
	OpenCV REQUIRED
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES camera
	CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
	DEPENDS OpenCV system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(
	camera_node
	src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	camera_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	/usr/lib/uv4l/uv4lext/armv6l/libuv4lext.so
)
