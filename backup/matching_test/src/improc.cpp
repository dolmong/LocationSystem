#include "improc.h"

CImproc::CImproc()
{
}


CImproc::~CImproc()
{
}


void CImproc::Reverse(
	sensor_msgs::Image &image
	)
{
	sensor_msgs::Image imgCpy=image;
	imgCpy.data.clear();

	int w=imgCpy.width, h=imgCpy.height;
	register int i=0, j=0;

	for( j=0; j<h; j++ )
	{
		for( i=0; i<w; i++ )
		{
			imgCpy.data.push_back( 255-image.data.at(j*w+i) );
		}
	}
	image = imgCpy;
}


void CImproc::RGB2Y(
	sensor_msgs::Image &image
	)
{
	sensor_msgs::Image imgCpy=image;
	imgCpy.data.clear();
	imgCpy.encoding=sensor_msgs::image_encodings::MONO8;
	register int i=0, j=0, k=0;
	register int w=image.width, h=image.height;

	register float R=0.f, G=0.f, B=0.f;
	register unsigned int Y=0;
	for( j=0; j<h; j++ )
	{
		for( i=0; i<w*3; i+=3 )
		{
			R = (float)image.data.at(j*w+i+2);
			G = (float)image.data.at(j*w+i+1);
			B = (float)image.data.at(j*w+i);
			Y = (unsigned int)( 0.299f*R+0.587f*G+0.114f*B+0.5);
			imgCpy.data.push_back( Y>255 ? 255:(unsigned char)Y );
		}
	}
	image.data.clear();
	image=imgCpy;
}
