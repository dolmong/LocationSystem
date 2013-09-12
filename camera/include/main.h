#ifndef MAIN_H
#define MAIN_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


class CMain
{
public:
	// Construct and Destruct
	CMain();
	~CMain();

public:
	// Function
	void Capture( int width, int height );

private:
	// Variable
	sensor_msgs::Image imgCap;
	cv_bridge::CvImagePtr cvimgCap;
	cv_bridge::CvImagePtr cvimgGray;
};

#endif
