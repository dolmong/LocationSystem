#ifndef LOCATIONSYSTEM_PROJECT_LOCATIONSYSTEM_H
#define LOCATIONSYSTEM_PROJECT_LOCATIONSYSTEM_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

class CLocationsystem
{
public:
	CLocationsystem();
	~CLocationsystem();

public:
	void CaptureCallback( const sensor_msgs::ImageConstPtr &msg );
	
private:
	ros::NodeHandle	nh_;
	image_transport::ImageTransport	it_;
	image_transport::Subscriber	image_sub_cap_;

private:
	sensor_msgs::Image	image;

private:
	void Reverse( sensor_msgs::Image &image );
	void ViewImage( sensor_msgs::Image image, const char *name );
};

#endif
