#ifndef SURF_TEST_PROJECT_MAIN_H
#define SURF_TEST_PROJECT_MAIN_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "surf/surf.h"
#include "filesystem/filesystem.h"
#include "viewer/viewer.h"

class CMain
{
public:
	CMain();
	~CMain();

public:
	CSURF surf;
	CFileSystem filesystem;
	CViewer viewer;
};

#endif
