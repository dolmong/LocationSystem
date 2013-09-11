#include "main.h"
#include <sstream>

CMain::CMain()
{
}


CMain::~CMain()
{
}


int main(
	int argc,
	char **argv
	)
{
	ros::init( argc, argv, "Main" );

	CMain main;

	sensor_msgs::Image image;
	std::string strPath="/home/pi/Jeong_ws/src/surf_test/picture";
	std::string strName="image.bmp";
	std::string strFile=main.filesystem.MergeFileName( strPath.c_str(), strName.c_str() );
	if( !main.filesystem.ReadImage( image, strFile.c_str() ) )
	{
		ROS_ERROR( "Main: ReadImage Fail!" );
		return 0;
	}
	else
	{
		main.surf.ipSurf.clear();
		main.surf.Surf( image, main.surf.ipSurf );

		std::string strAdd="_surf.txt";
		strName = main.filesystem.AddFileName( strName.c_str(), strAdd.c_str() );
		strFile = main.filesystem.MergeFileName( strPath.c_str(), strName.c_str() );
		ROS_INFO( strFile.c_str() );
		if( !main.filesystem.WriteSurf(main.surf.ipSurf, strFile.c_str()) )
		{
			ROS_ERROR( "Main: WriteSurf Fail!" );
			return 0;
		}

		// main.viewer.ViewImage( image );

		image.data.clear();
	}
	ROS_INFO( "surf test end" );
	return 0;
}
