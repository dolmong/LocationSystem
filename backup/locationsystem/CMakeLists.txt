cmake_minimum_required( VERSION 2.8.3 )
project( locationsystem )

find_package(
	catkin REQUIRED
	COMPONENTS cv_bridge image_transport roscpp sensor_msgs genmsg
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
	LIBRARIES picamcv
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
include_directories(
	/opt/vc/userland/host_applications/linux/libs/bcm_host/include
	/opt/vc/userland/interface/vcos
	/opt/vc/userland
	/opt/vc/userland/interface/vcos/pthreads
	/opt/vc/userland/interface/vmcs_host/linux
)

## Declare a cpp executable
add_executable(
	locationsystem_node
	src/picam/RaspiCamControl.c
	src/picam/RaspiCLI.c
	src/picam/RaspiPreview.c
	src/improc/improc.cpp
	src/surf/Matrix.cpp
	src/surf/SURF.cpp
	src/filesystem/filesystem.cpp
	src/main.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
	locationsystem_node
	picamcv_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	locationsystem_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	/opt/vc/lib/libmmal_core.so
	/opt/vc/lib/libmmal_util.so
	/opt/vc/lib/libmmal_vc_client.so
	/opt/vc/lib/libvcos.so
	/opt/vc/lib/libbcm_host.so
)
