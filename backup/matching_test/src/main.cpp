#include "main.h"

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

	sensor_msgs::Image imgRef;
	std::string strRefPath="/home/pi/picture/Database";
	std::string strRefName="(0,0).bmp";
	std::string strRefFile=main.filesystem.MergeFileName( strRefPath.c_str(), strRefName.c_str() );
	if( !main.filesystem.ReadImage( imgRef, strRefFile.c_str() ) )
	{
		ROS_ERROR( "Main: ReadImage_ReferenceImage Fail!" );
		return 0;
	}
	else
	{
		main.surfRef.ipSurf.clear();
		std::string strAdd="_surf.txt";
                std::string strRefSurfName=main.filesystem.AddFileName( strRefName.c_str(), strAdd.c_str() );
                std::string strRefSurfFile=main.filesystem.MergeFileName( strRefPath.c_str(), strRefSurfName.c_str() );

		if( !main.filesystem.ReadSurf(main.surfRef.ipSurf, strRefSurfFile.c_str()) )
		{
			main.surfRef.Surf( imgRef, main.surfRef.ipSurf );

			if( !main.filesystem.WriteSurf(main.surfRef.ipSurf, strRefSurfFile.c_str()) )
			{
				ROS_ERROR( "Main: WriteSurf Fail!" );
				imgRef.data.clear();
				return 0;
			}
		}

		sensor_msgs::Image imgIn;
		std::string strInPath=strRefPath.c_str();
		std::string strInName="(0,1).bmp";
		std::string strInFile=main.filesystem.MergeFileName( strInPath.c_str(), strInName.c_str() );
		if( !main.filesystem.ReadImage(imgIn, strInFile.c_str()) )
		{
			ROS_ERROR( "Main: ReadImage_InputImage Fail!" );
		}
		main.surfIn.Surf( imgIn, main.surfIn.ipSurf );

		strAdd = "_HEIGHT.bmp";
		std::string strRefHeightName=main.filesystem.AddFileName( strRefName.c_str(), strAdd.c_str() );
		std::string strRefHeightFile=main.filesystem.MergeFileName( strRefPath.c_str(), strRefHeightName.c_str() );
		
		main.matching.Matching( &imgRef, &imgIn, &main.surfRef.ipSurf, &main.surfIn.ipSurf, &strRefHeightFile, main.matching.ipPairMat, main.matching.spMeter );
		imgRef.data.clear();
		imgIn.data.clear();
	}

	ROS_INFO( "Matching test end" );

	return 0;
}
