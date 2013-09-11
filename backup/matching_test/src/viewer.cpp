#include "viewer.h"

namespace enc = sensor_msgs::image_encodings;

static const char Window[] = "Image";

CViewer::CViewer()
{
	cv::namedWindow( Window, CV_WINDOW_AUTOSIZE );
}


CViewer::~CViewer()
{
	cv::destroyWindow( Window );
}


bool CViewer::ViewImage(
	sensor_msgs::Image image
	)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr.reset( new cv_bridge::CvImage );
	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, enc::MONO8 );
		image.data.clear();
	}
	catch( cv_bridge::Exception &e )
	{
		ROS_ERROR( "cv_bridge excepion: [%s]", e.what() );
		return false;
	}

	ROS_INFO( "width: [%d], height: [%d]",cv_ptr->image.cols, cv_ptr->image.rows );

	cv::imshow( Window, cv_ptr->image );
	cv::waitKey(0);
	cv_ptr->image.release();

	return true;
}
