#include "locationsystem/locationsystem.h"

namespace enc = sensor_msgs::image_encodings;

static const char CapWindow[] = "Capture Window";
static const char RevWindow[] = "Reverse Window";

CLocationsystem::CLocationsystem()
:	it_( nh_ )
{
	ROS_INFO( "LocationSystem Start" );
	image_sub_cap_ = it_.subscribe( "Capture_Out", 1, &CLocationsystem::CaptureCallback, this );

	// cv::namedWindow( CapWindow, CV_WINDOW_AUTOSIZE );
	// cv::namedWIndow( RevWIndow, CV_WINDOW_AUTOSIZE );
}


CLocationsystem::~CLocationsystem()
{
	ROS_INFO( "LocationSystem End" );

	// cv::destroyWindow( CapWindow );
	// cv::destroyWindow( RevWindow );
}


void CLocationsystem::CaptureCallback(
	const sensor_msgs::ImageConstPtr &msg
	)
{
	image=*msg;

	sensor_msgs::Image imgRev=image;

	Reverse( imgRev );
	ROS_INFO( "Rev Test Complete" );

	// ViewImage( imgRev, RevWindow );
}


void CLocationsystem::Reverse(
	sensor_msgs::Image &image
	)
{
	sensor_msgs::Image imgCpy=image;

	imgCpy.data.clear();
	int w=imgCpy.width, h=imgCpy.height;

	for( int j=0; j<h; j++ )
	{
		for( int i=0; i<w; i++ )
		{
			imgCpy.data.push_back( 255-image.data.at(j*w+i) );
		}
	}
	image=imgCpy;
}


void CLocationsystem::ViewImage(
	sensor_msgs::Image image,
	const char *name
	)
{
	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvCopy( image, enc::MONO8 );
	}
	catch( cv_bridge::Exception &e )
	{
		ROS_ERROR( "cv_bridge excepion: [%s]", e.what() );
		return;
	}

	cv::imshow( name, cv_ptr->image );
	cv::waitKey( 1 );
}


int main(
	int argc,
	char **argv
	)
{
	ros::init( argc, argv, "LocationSystem" );

	CLocationsystem ls;

	ros::spin();

	return 0;
}
