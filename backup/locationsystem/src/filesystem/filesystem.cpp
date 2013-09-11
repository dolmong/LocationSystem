#include "filesystem/filesystem.h"

namespace enc=sensor_msgs::image_encodings;

CFileSystem::CFileSystem()
{
}


CFileSystem::~CFileSystem()
{
}


bool CFileSystem::WriteImage(
	sensor_msgs::Image image,
	std::string strName
	)
{
	ROS_INFO( "width: [%d], height: [%d], enc: [%s], file: [%s]", image.width, image.height, image.encoding.c_str(), strName.c_str() );
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, enc::MONO8 );
		image.data.clear();
	}
	catch( cv_bridge::Exception &e )
	{
		ROS_ERROR( "cv_bridge exception: [%s]", e.what() );
		return false;
	}

	cv::imwrite( strName.c_str(), cv_ptr->image );
	cv_ptr->image.release();

	return true;
}


bool CFileSystem::ReadImage(
	sensor_msgs::Image &image,
	std::string strName
	)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr.reset( new cv_bridge::CvImage );
	ROS_INFO( "file name: [%s]", strName.c_str() );
	cv_ptr->image = cv::imread( strName.c_str() );
	if( cv_ptr->image.empty()!=false )
	{
		ROS_ERROR( "Image read fail!" );
		return false;
	}
	cv_ptr->encoding = enc::MONO8;
	sensor_msgs::ImagePtr pimg = cv_ptr->toImageMsg();
	if( pimg->data.size()==0 )
	{
		ROS_ERROR( "cv::Mat to sensor_msgs::ImagePtr Copy fail!" );
		cv_ptr->image.release();
		return false;
	}
	cv_ptr->image.release();

	image = *pimg;

	ROS_INFO( "width: [%d], height: [%d], enc: [%s]", image.width, image.height, image.encoding.c_str() );
	return true;
}
