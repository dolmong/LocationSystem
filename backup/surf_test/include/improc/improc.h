#ifndef IMPROC_PROJECT_IMPROC_H
#define IMPROC_PROJECT_IMPROC_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class CImproc
{
public:
	CImproc();
	~CImproc();

	void Reverse( sensor_msgs::Image &image );
	void RGB2Y( sensor_msgs::Image &image );
};

#endif
