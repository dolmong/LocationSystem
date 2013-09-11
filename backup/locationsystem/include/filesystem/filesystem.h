#ifndef LOCATIONSYSTEM_PROJECT_FILESYSTEM_H
#define LOCATIONSYSTEM_PROJECT_FILESYSTEM_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>

class CFileSystem
{
public:
	CFileSystem();
	~CFileSystem();

	bool WriteImage( sensor_msgs::Image image, std::string strName );
	bool ReadImage( sensor_msgs::Image &image, std::string strName );
};

#endif
