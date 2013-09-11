#include "capture.h"

CCapture::CCapture(
	)
	:it_( nh_ )
{
	image_pub_ = it_.advertise( "Capture_Out", 1 );
	cvImage_ptr.reset( new cv_bridge::CvImage );
}

CCapture::~CCapture()
{
//	cv::destroyWindow( WINDOW );
	cvImage_ptr->image.release();
}

bool CCapture::Capture(
	sensor_msgs::Image &img
	)
{
/*
	cv::VideoCapture video( 0 );
	if( !video.isOpened() )
	{
		ROS_ERROR( "CCapture(Capture): No camera detected!" );
		return false;
	}
	ROS_INFO( "In Capture ..." );

	video.set( CV_CAP_PROP_FRAME_WIDTH, 1280 );
	video.set( CV_CAP_PROP_FRAME_HEIGHT, 720 );

	cv::namedWindow( WINDOW, CV_WINDOW_AUTOSIZE );
	while( 1 )
	{
		cv::Mat frame;
		bool bSuccess=video.read( frame );

		if( !bSuccess )
		{
			ROS_ERROR( "CCapture(Capture): Can't read a frame from camera" );
			return false;
		}
	
		cv::imshow( WINDOW, frame );

		if( cv::waitKey(30)==27 )
		{
			ROS_INFO( "Exit" );
			break;
		}
	}
*/
	CvCapture *capture=NULL;
	cv::Mat frame;

	capture = cvCreateCameraCapture( 0 );	// 0=defalt, -1=any camera, 1~99=your camera

	if( !capture )
	{
		ROS_ERROR( "CCapture(Capture): No camera detected!" );
		return false;
	}
	else
	{
		ROS_INFO( "In capture ..." );

		while( 1 )
		{
			IplImage *ipImg=cvQueryFrame( capture );
			frame = ipImg;
			if( frame.empty() )
			{
				ROS_ERROR( "CCapture(Capture): Don't capture!" );
				cvReleaseCapture( &capture );
				cvReleaseImage( &ipImg );
				frame.release();
				return false;
			}
			if( ipImg->origin==IPL_ORIGIN_TL )
				frame.copyTo( cvImage_ptr->image );
			else
				cv::flip( frame, cvImage_ptr->image, 0 );

			img.data.clear();
			img.encoding = cvImage_ptr->encoding = sensor_msgs::image_encodings::MONO8;
			cvImage_ptr->toImageMsg( img );

//			imshow( WINDOW, cvImage_ptr->image );

			if( cv::waitKey( 100 )>=0 )
			{
				cvReleaseCapture( &capture );
				cvReleaseImage( &ipImg );
				frame.release();
				ROS_ERROR( "CCapture(Capture): Time Out!" );
				return false;
			}
			cvReleaseImage( &ipImg );
		}
	}
	frame.release();

	return true;
}

int main(
	int argc,
	char **argv
	)
{
	ros::init( argc, argv, "Capture" );

	CCapture capture;

	sensor_msgs::Image imgOrg;

	ROS_INFO( "capture start" );
	while( ros::ok() )
	{
		if( !capture.Capture(imgOrg) )
		{
			ROS_ERROR( "Capture Fail!" );
			break;
		}
		else
		{
			ROS_INFO( "Capture Success!" );
			break;
		}
	}

	return 0;
}
