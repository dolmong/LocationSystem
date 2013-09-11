#include "viewer/viewer.h"

namespace enc = sensor_msgs::image_encodings;

static const char Window[] = "Image";

CViewer::CViewer()
:	it_( nh_ )
{
	image_sub_view_ = it_.subscribe( "Image", 1, &CViewer::ViewerCallback, this );

	cv::namedWindow( Window, CV_WINDOW_AUTOSIZE );
}


CViewer::~CViewer()
{
	cv::destroyWindow( Window );
}


void CViewer::ViewerCallback(
	const sensor_msgs::ImageConstPtr &msg
	)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy( msg, enc::MONO8 );
	}
	catch( cv_bridge::Exception &e )
	{
		ROS_ERROR( "cv_bridge excepion: [%s]", e.what() );
		return;
	}

	cv::imshow( Window, cv_ptr->image );
	cv::waitKey( 1 );
}


int main(
	int argc,
	char **argv
	)
{
	ros::init( argc, argv, "Viewer" );

	CViewer view;

	ros::spin();

	return 0;
}
