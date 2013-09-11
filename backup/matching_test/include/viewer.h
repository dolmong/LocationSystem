#ifndef LOCATIONSYSTEM_PROJECT_LOCATIONSYSTEM_H
#define LOCATIONSYSTEM_PROJECT_LOCATIONSYSTEM_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

class CViewer
{
public:
	CViewer();
	~CViewer();

public:
	bool ViewImage( sensor_msgs::Image image );
};

#endif
