
#ifndef PICAMCV_PROJECT_CAMCV_H
#define PICAMCV_PROJECT_CAMCV_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class CCapture
{
public:
	CCapture( );
	~CCaptuer();

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;

	cv_bridge::CvImagePtr cvImage_ptr;
}


#endif
