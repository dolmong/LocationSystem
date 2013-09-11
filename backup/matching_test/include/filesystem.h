#ifndef FILESYSTEM_CLASS_FILESYSTEM_H
#define FILESYSTEM_CLASS_FILESYSTEM_H

#include <string>
#include <cstdio>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>

#include "global.h"


class CFileSystem
{
public:
	CFileSystem();
	~CFileSystem();

public:
	bool WriteImage( sensor_msgs::Image image, std::string strName );
	bool ReadImage( sensor_msgs::Image &image, std::string strName );

	bool ReadSurf( std::vector<Interestpoint> &ipSurf, std::string strName );
	bool WriteSurf( std::vector<Interestpoint> ipSurf,std::string strName );

//	std::string *SplitFileName( std::string strPath, std::string strName );
	std::string MergeFileName( std::string strPath, std::string strName );

	std::string AddFileName( std::string strName, std::string strAdd );
};

#endif
