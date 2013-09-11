#ifndef CAMERA_CLASS_CAPTURE_H
#define CAMERA_CLASS_CAPTURE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

static const char WINDOW[] = "Image window";

class CCapture
{
public:
	CCapture();
	~CCapture();

	bool Capture( sensor_msgs::Image &img );
private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;

	cv_bridge::CvImagePtr cvImage_ptr;
};

#endif
