#include "locationsystem/improc.h"

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
