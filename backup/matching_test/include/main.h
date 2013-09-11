#ifndef SURF_TEST_PROJECT_MAIN_H
#define SURF_TEST_PROJECT_MAIN_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "surf.h"
#include "filesystem.h"
// #include "viewer.h"
#include "matching.h"

class CMain
{
public:
	CMain();
	~CMain();

public:
	CSURF surfRef;
	CSURF surfIn;
	CMatching matching;
	CFileSystem filesystem;
	// CViewer viewer;
};

#endif
