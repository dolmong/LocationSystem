#include "surf.h"

CSURF::CSURF(void)
	: m_fthreshold(0.f)
	, m_pIntegralImage(NULL)
{
	memset(m_layer, 0x00, sizeof(m_layer));
}


CSURF::~CSURF(void)
{
	if( m_pIntegralImage!=NULL )
	{
		free( m_pIntegralImage );
		m_pIntegralImage=NULL;
	}
	if( m_vInterestpoint.size()!=0 )
		m_vInterestpoint.clear();
	if( ipSurf.size()!=0 )
		ipSurf.clear();
	
}


inline int CSURF::fRound(
	float flt
	)
{
	return (int)floor( flt+0.5f );
}


void CSURF::CalculateIntgral(
	sensor_msgs::Image image
	)
{
	int w=image.width, h=image.height;

	int i=0, j=0;

	float* GrayImage=(float*)malloc( sizeof(float)*w*h );
	memset( GrayImage, 0x00, sizeof(float)*w*h );

	for( j=0; j<h; j++)
	{
		for(  i=0; i<w; i++)
		{
			GrayImage[j*w+i] = (float)( image.data.at(j*w+i) )/255.f;
		}
	}

	float rs=0.f;
	for( i=0; i<w; i++) 
	{
		rs+=GrayImage[i]; 
		m_pIntegralImage[i] = rs;
	}
	for( j=1; j<h; j++)
	{
		rs = 0.0f;
		for( i=0; i<w; i++) 
		{
			rs += GrayImage[j*w+i]; 
			m_pIntegralImage[j*w+i]=rs+m_pIntegralImage[(j-1)*w+i];
		}
	}

	if( GrayImage!=NULL )
	{
		free( GrayImage );
		GrayImage=NULL;
	}
}


float CSURF::CalculateRectangularSum(
	int y,
	int x,
	int cy,
	int cx
	)
{
	float A=0.f, B=0.f, C=0.f, D=0.f;
	int r1=y<m_nheight ? y-1:m_nheight-1;
	int c1=x<m_nwidth ? x-1:m_nwidth-1;
	int r2=y+cy<m_nheight ? y+cy-1:m_nheight-1;
	int c2=x+cx<m_nwidth ? x+cx-1:m_nwidth-1;

	if( r1>=0 && c1>=0 )
		A = m_pIntegralImage[r1*m_nwidth+c1];
	if( r1>=0 && c2>=0 )
		B = m_pIntegralImage[r1*m_nwidth+c2];
	if( r2>=0 && c1>=0 )
		C = m_pIntegralImage[r2*m_nwidth+c1];
	if( r2>=0 && c2>=0 )
		D = m_pIntegralImage[r2*m_nwidth+c2];

	return 0.f>A-B-C+D ? 0.f:A-B-C+D;
}


void CSURF::ConstructScaleSpace()
{
	for( int o=0; o<Octave_Num; o++ )
	{
		int interval=2<<o;
		int cx=m_nwidth/interval;
		int cy=m_nheight/interval;

		for( int l=0; l<Layer_Num; l++ )
		{
			m_layer[o][l]._filter = 3*( (int)pow(2.f, (o+1))*(l+1)+1 );
			m_layer[o][l]._width = cx;
			m_layer[o][l]._height = cy;
			m_layer[o][l]._interval = interval;
			m_layer[o][l]._pdetH = new float[sizeof(float)*cx*cy];
			m_layer[o][l]._ptrace = new float[sizeof(float)*cx*cy]; 

			memset( m_layer[o][l]._pdetH, 0x00, sizeof(float)*cx*cy );
			memset( m_layer[o][l]._ptrace, 0x00, sizeof(float)*cx*cy );

			CalulateFastHessian( o, l );
		}
	}
}


void CSURF::CalulateFastHessian(
	int octave,
	int layer
	)
{

	int interval = m_layer[octave][layer]._interval;
	int filter   = m_layer[octave][layer]._filter;
	int width    = m_layer[octave][layer]._width;
	int height   = m_layer[octave][layer]._height;

	int border = (filter - 1) / 2 + 1;          
	int lobes  =  filter / 3;   

	int index = 0;	
	float Dxx, Dyy, Dxy;

	for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			int y = i * interval;
			int x = j * interval;

			Dxx = CalculateRectangularSum(y - lobes + 1, x - border, 2*lobes - 1, filter)
				- CalculateRectangularSum(y - lobes + 1, x - lobes/2, 2*lobes - 1, lobes)*3;

			Dyy = CalculateRectangularSum(y - border,  x - lobes + 1, filter, 2*lobes -1)
				- CalculateRectangularSum(y - lobes/2, x - lobes + 1, lobes,  2*lobes -1)*3;

			Dxy = CalculateRectangularSum(y - lobes, x + 1,     lobes, lobes)
				+ CalculateRectangularSum(y + 1,     x - lobes, lobes, lobes)
				- CalculateRectangularSum(y - lobes, x - lobes, lobes, lobes)
				- CalculateRectangularSum(y + 1,     x + 1,     lobes, lobes);	

			Dxx /=  (filter * filter);
			Dyy /=  (filter * filter);
			Dxy /=  (filter * filter);

			m_layer[octave][layer]._pdetH[index]  = ((Dxx * Dyy) - (0.81f * Dxy * Dxy));
			m_layer[octave][layer]._ptrace[index] = (Dxx + Dyy >= 0 ? 1.f:0.f);		

			index++;
		}
	}
}


void CSURF::LocaliseInterestPoint(
	float threshold
	)
{
	m_fthreshold = threshold;
	m_vInterestpoint.clear();

	for( int nOctave=0; nOctave<Octave_Num; nOctave++ )
		for( int nLayer=1; nLayer<Layer_Num-1; nLayer++ )
		{

			int width  = m_layer[nOctave][nLayer]._width;
			int height = m_layer[nOctave][nLayer]._height;

			for( int y=0; y<height; ++y )
				for( int x=0; x<width; ++x )
				{
					if( x==58 && y==8 )
						int xy=0;

					if( ApplyNonMaximalSuppression(nOctave, nLayer, y, x) ) 
						continue;
					else
						InterpolateInterestpoint( nOctave, nLayer, x, y );
				}
		}

		for( int nOctave=0; nOctave<Octave_Num; nOctave++ )
			for( int nLayer=0; nLayer<Layer_Num; nLayer++ )
			{
				delete [] m_layer[nOctave][nLayer]._pdetH;
				delete [] m_layer[nOctave][nLayer]._ptrace;
			}
}

bool CSURF::ApplyNonMaximalSuppression(
	int octave,
	int layer,
	int y,
	int x
	)
{

	bool morethancandiate = false;

	Responselayer *below  = &m_layer[octave][layer-1];
	Responselayer *native = &m_layer[octave][layer];
	Responselayer *above  = &m_layer[octave][layer+1];

	int layerBorder = (above->_filter + 1) / (2 * above->_interval);
	if( y<=layerBorder || y>=above->_height-layerBorder || x<=layerBorder || x>=above->_width-layerBorder )
	{
		return morethancandiate = true;
	}

	int scale = native->_width / above->_width;
	float candiate = native->_pdetH[(y*scale) * native->_width + (x*scale)];

	if(candiate < m_fthreshold) 
		return morethancandiate = true;

	int scale_b = below->_width / above->_width;

	for( int i=-1; i<=1; ++i )
	{
		for( int j=-1; j<=1; ++j )
		{
			if( above->_pdetH[(y+i)*above->_width+(x+j)]>=candiate
				|| (native->_pdetH[((y+i)*scale)*native->_width+((x+j)*scale)]>=candiate && (i!=0||j!=0))
				||	below->_pdetH[((y+i)*scale_b)*below->_width+((x+j)*scale_b)]>=candiate
				)
			{
				morethancandiate = true;
				break;
			}
		}
	}

	return morethancandiate;
}


void CSURF::InterpolateInterestpoint(
	int octave,
	int layer,
	int x,
	int y
	)
{
	Responselayer* below  = &m_layer[octave][layer-1];
	Responselayer* native = &m_layer[octave][layer];
	Responselayer* above  = &m_layer[octave][layer+1];

	float lx = 0.f;
	float ly = 0.f;
	float ls = 0.f;

	static int count_Local =0;

	int scale_n=native->_width/above->_width;
	int scale_b=below->_width/above->_width;

	float dx=( native->_pdetH[(y*scale_n*native->_width)+((x+1)*scale_n)] 
		-native->_pdetH[(y*scale_n*native->_width)+((x-1)*scale_n)] )/2.f;

	float dy=( native->_pdetH[((y+1)*scale_n*native->_width)+(x*scale_n)] 
		-native->_pdetH[((y-1)*scale_n*native->_width)+(x*scale_n)] )/2.f;

	float ds=( above->_pdetH[y*above->_width+x]-below->_pdetH[(y*scale_n*below->_width)+(x*scale_b)] )/2.f;


	float dxx=native->_pdetH[(y*scale_n*native->_width)+((x+1)*scale_n)]
		+native->_pdetH[(y*scale_n*native->_width)+((x-1)*scale_n)]
		-2*native->_pdetH[(y*scale_n*native->_width)+(x*scale_n)];

	float dyy=native->_pdetH[((y+1)*scale_n*native->_width)+(x*scale_n)]
		+native->_pdetH[((y-1)*scale_n*native->_width)+(x*scale_n)]
		-2*native->_pdetH[(y*scale_n*native->_width) +(x*scale_n)];

	float dss=above->_pdetH[(y*above->_width)+x]
		+below->_pdetH[(y*scale_n*below->_width)
		+(x*scale_n)]-2*native->_pdetH[(y*scale_n*native->_width)+(x*scale_b)];

	float dxy=( native->_pdetH[((y+1)*scale_n*native->_width)+((x+1)*scale_n)]
		-native->_pdetH[((y+1)*scale_n*native->_width)+((x-1)*scale_n)]
		-native->_pdetH[((y-1)*scale_n*native->_width)+((x+1)*scale_n)]
		+native->_pdetH[((y-1)*scale_n*native->_width)+((x-1)*scale_n)] )/4.f;

	float dxs=( above->_pdetH[(y*above->_width)+(x+1)]
		-above->_pdetH[(y*above->_width)+(x-1)]
		-below->_pdetH[(y*scale_b*below->_width)+((x+1)*scale_b)]
		+below->_pdetH[(y*scale_b*below->_width)+((x-1)*scale_b)] )/4.f;

	float dys=( above->_pdetH[((y+1)*above->_width)+x]
		-above->_pdetH[((y-1)*above->_width)+x]
		-below->_pdetH[((y+1)*scale_b*below->_width)+(x*scale_b)]
		+below->_pdetH[((y-1)*scale_b*below->_width)+(x*scale_b)] )/4.f;

	#define N 3
	
	float  **a, **yy, d, *col;
	int i, j, *indx;

	a = matrix_allocate( 1, 3, 1, 3 );
	yy = matrix_allocate( 1, 3, 1, 3 );
	indx = ivector( 1, 3 );
	col = vector_allocate( 1, 3 );


	a[1][1] = dxx;	a[1][2] = dxy;	a[1][3] = dxs;
	a[2][1] = dxy;	a[2][2] = dyy;	a[2][3] = dys;
	a[3][1] = dxs;	a[3][2] = dys;	a[3][3] = dss;

	ludcmp(a,N,indx, &d);

	for (j=1; j<=N; j++)
	{
		for(i=1; i<=N; i++) col[i]=0.0;
		col[j]=1.0;
		lubksb(a,N,indx,col);
		for(i=1; i<=N; i++)
			yy[i][j]=col[i];
	}

	lx = -(yy[1][1]*dx + yy[1][2]*dy + yy[1][3]*ds);
	ly = -(yy[2][1]*dx + yy[2][2]*dy + yy[2][3]*ds);
	ls = -(yy[3][1]*dx + yy[3][2]*dy + yy[3][3]*ds);

	free_matrix(a,1,3,1,3);		free_matrix(yy,1,3,1,3);	
	free_ivector(indx,1,3);		free_vector(col, 1, 3);

	float s = (float)(native->_filter)*(1.2f/9.f);
	int interval = native->_filter-below->_filter;
	if( fabs(lx)<0.5f && fabs(ly)<0.5f && fabs(ls)<0.5f )
	{
		static int count = 0;

		Interestpoint ipt;
		ipt._x = static_cast<float>( (x+lx)*native->_interval );
		ipt._y = static_cast<float>( (y+ly)*native->_interval );
		ipt._scale = static_cast<float>( (0.1333f)*(native->_filter+ls*interval) );
		ipt._Lap = (int)( native->_ptrace[(scale_n*y)*native->_width+(scale_n*x)] );
		ipt._majordiretion = 0.f;
		ipt._filter = native->_filter;
		ipt._chkOverlap = 0;

		m_vInterestpoint.push_back( ipt );
		// ROS_INFO( "%03d : %30f, %30f\n", count++, ipt._x, ipt._y );		
	}
	else
		count_Local++;
}

void CSURF::GetInterestPoint(
	std::vector<Interestpoint> &ipTmp
	)
{
	ipTmp = m_vInterestpoint;
}


void CSURF::getDescriptors(
	std::vector<Interestpoint> &vItpoint
	)
{
	int ipts_size = vItpoint.size();

	for( int i=0; i<ipts_size; i++ )
	{
		des_index = i;
		getOrientation( vItpoint );
		getDescriptor( vItpoint );
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const double gauss25 [7][7] = {
	0.02350693969273,	0.01849121369071,	0.01239503121241,	0.00708015417522,	0.00344628101733,	0.00142945847484,	0.00050524879060,
	0.02169964028389,	0.01706954162243,	0.01144205592615,	0.00653580605408,	0.00318131834134,	0.00131955648461,	0.00046640341759,
	0.01706954162243,	0.01342737701584,	0.00900063997939,	0.00514124713667,	0.00250251364222,	0.00103799989504,	0.00036688592278,
	0.01144205592615,	0.00900063997939,	0.00603330940534,	0.00344628101733,	0.00167748505986,	0.00069579213743,	0.00024593098864,
	0.00653580605408,	0.00514124713667,	0.00344628101733,	0.00196854695367,	0.00095819467066,	0.00039744277546,	0.00014047800980,
	0.00318131834134,	0.00250251364222,	0.00167748505986,	0.00095819467066,	0.00046640341759,	0.00019345616757,	0.00006837798818,
	0.00131955648461,	0.00103799989504,	0.00069579213743,	0.00039744277546,	0.00019345616757,	0.00008024231247,	0.00002836202103
};

const double gauss33 [11][11] = {
	0.014614763,	0.013958917,	0.012162744,	0.00966788,	0.00701053,	0.004637568,	0.002798657,	0.001540738,	0.000773799,	0.000354525,	0.000148179,
	0.013958917,	0.013332502,	0.011616933,	0.009234028,	0.006695928,	0.004429455,	0.002673066,	0.001471597,	0.000739074,	0.000338616,	0.000141529,
	0.012162744,	0.011616933,	0.010122116,	0.008045833,	0.005834325,	0.003859491,	0.002329107,	0.001282238,	0.000643973,	0.000295044,	0.000123318,
	0.00966788,	0.009234028,	0.008045833,	0.006395444,	0.004637568,	0.003067819,	0.001851353,	0.001019221,	0.000511879,	0.000234524,	9.80224E-05,
	0.00701053,	0.006695928,	0.005834325,	0.004637568,	0.003362869,	0.002224587,	0.001342483,	0.000739074,	0.000371182,	0.000170062,	7.10796E-05,
	0.004637568,	0.004429455,	0.003859491,	0.003067819,	0.002224587,	0.001471597,	0.000888072,	0.000488908,	0.000245542,	0.000112498,	4.70202E-05,
	0.002798657,	0.002673066,	0.002329107,	0.001851353,	0.001342483,	0.000888072,	0.000535929,	0.000295044,	0.000148179,	6.78899E-05,	2.83755E-05,
	0.001540738,	0.001471597,	0.001282238,	0.001019221,	0.000739074,	0.000488908,	0.000295044,	0.00016243,	8.15765E-05,	3.73753E-05,	1.56215E-05,
	0.000773799,	0.000739074,	0.000643973,	0.000511879,	0.000371182,	0.000245542,	0.000148179,	8.15765E-05,	4.09698E-05,	1.87708E-05,	7.84553E-06,
	0.000354525,	0.000338616,	0.000295044,	0.000234524,	0.000170062,	0.000112498,	6.78899E-05,	3.73753E-05,	1.87708E-05,	8.60008E-06,	3.59452E-06,
	0.000148179,	0.000141529,	0.000123318,	9.80224E-05,	7.10796E-05,	4.70202E-05,	2.83755E-05,	1.56215E-05,	7.84553E-06,	3.59452E-06,	1.50238E-06
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CSURF::getOrientation(
	std::vector<Interestpoint> &vItpoint
	)
{
	Interestpoint *ipt=&vItpoint[des_index];
	float gauss=0.f;
	float scale=ipt->_scale;
	const int s=fRound(scale);
	const int r=fRound(ipt->_y);
	const int c=fRound(ipt->_x);
	std::vector<float> resX( 109 ), resY( 109 ), Ang( 109 );
	const int id[]={6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6};

	int idx=0;
	// calculate haar responses for points within radius of 6*scale
	for( int i=-6; i<=6; ++i )
	{
		for( int j=-6; j<=6; j++ )
		{
			if( i*i+j*j<36 )
			{
				gauss = static_cast<float>( gauss25[id[i+6]][id[j+6]] );
				resX[idx] = gauss * haarX( r+j*s, c+i*s, 4*s );
				resY[idx] = gauss * haarY( r+j*s, c+i*s, 4*s );
				Ang[idx] = getAngle( resX[idx], resY[idx] );
				++idx;
			}

		}
	}

	// calculate the dominant direction 
	float sumX = 0.f, sumY = 0.f;
	float max = 0.f, orientation = 0.f;
	float ang1 = 0.f, ang2 = 0.f;

	// loop slides pi/3 window around feature point
	for( ang1=0; ang1<2*M_PI;  ang1+=0.15f )
	{
		ang2 = (float)( ang1+M_PI/3.0f>2*M_PI ? ang1-5.0f*M_PI/3.0f : ang1+M_PI/3.0f );
		sumX = sumY = 0.f; 
		for( unsigned int k=0; k<Ang.size(); ++k ) 
		{
			// get angle from the x-axis of the sample point
			const float &ang = Ang[k];

			// determine whether the point is within the window
			if( ang1<ang2 && ang1<ang && ang<ang2 ) 
			{
				sumX += resX[k];  
				sumY += resY[k];
			} 
			else if( ang2<ang1 && ((ang>0 && ang<ang2) || (ang>ang1 && ang<2*M_PI)) ) 
			{
				sumX += resX[k];  
				sumY += resY[k];
			}
		}

		// if the vector produced from this window is longer than all 
		// previous vectors then this forms the new dominant direction
		if( sumX*sumX+sumY*sumY>max ) 
		{
			// store largest orientation
			max = sumX*sumX+sumY*sumY;
			orientation = (float)getAngle( sumX, sumY );
		}
	}
	// assign orientation of the dominant response vector
	ipt->_majordiretion = orientation;	
}

//! Modified descriptor contributed by Pablo Fernandez
void CSURF::getDescriptor(
	std::vector<Interestpoint> &vItpoint
	)
{
	int y, x, sample_x, sample_y, count = 0;
	int i = 0, ix = 0, j = 0, jx = 0, xs = 0, ys = 0;
	float scale, *desc, dx, dy, mdx, mdy, co, si;
	float gauss_s1 = 0.f, gauss_s2 = 0.f;
	float rx = 0.f, ry = 0.f, rrx = 0.f, rry = 0.f, len = 0.f;
	float cx = -0.5f, cy = 0.f;

	Interestpoint *ipt = &vItpoint[des_index];
	scale = ipt->_scale;
	x = fRound(ipt->_x);
	y = fRound(ipt->_y);  
	desc = ipt->_IpDescriptor;

	co = cos( ipt->_majordiretion );
	si = sin( ipt->_majordiretion );

	i = -8;

	while( i<12 )
	{
		j = -8;
		i = i-4;

		cx += 1.f;
		cy = -0.5f;

		while( j<12 ) 
		{
			dx = dy = mdx = mdy = 0.f;
			cy += 1.f;

			j = j - 4;

			ix = i + 5;
			jx = j + 5;

			xs = fRound( x+(-jx*scale*si+ix*scale*co) );
			ys = fRound( y+(jx*scale*co+ix*scale*si) );

			for(int k=i; k<i+9; ++k ) 
			{
				for(int l=j; l<j+9; ++l ) 
				{
					sample_x = fRound( x+(-l*scale*si+k*scale*co) );
					sample_y = fRound( y+(l*scale*co+k*scale*si) );

					gauss_s1 = gaussian( xs-sample_x, ys-sample_y, 2.5f*scale );
					rx = haarX( sample_y, sample_x, 2*fRound(scale) );
					ry = haarY( sample_y, sample_x, 2*fRound(scale) );

					rrx = gauss_s1*( -rx*si+ry*co );
					rry = gauss_s1*( rx*co+ry*si );

					dx += rrx;
					dy += rry;
					mdx += fabs( rrx );
					mdy += fabs( rry );

				}
			}

			//Add the values to the descriptor vector
			gauss_s2 = gaussian( cx-2.0f, cy-2.0f, 1.5f );

			desc[count++] = dx*gauss_s2;
			desc[count++] = dy*gauss_s2;
			desc[count++] = mdx*gauss_s2;
			desc[count++] = mdy*gauss_s2;

			len += (dx*dx+dy*dy+mdx*mdx+mdy*mdy)*gauss_s2*gauss_s2;

			j += 9;
		}
		i += 9;
	}	

	len = sqrt( len );
	for(int i=0; i<64; ++i )
	{
		desc[i] /= len;
	}
}


inline float CSURF::haarX(
	int row,
	int column,
	int s
	)
{
	return CalculateRectangularSum( row-s/2, column, s, s/2 )-1*CalculateRectangularSum( row-s/2, column-s/2, s, s/2 );
}


inline float CSURF::haarY(
	int row,
	int column,
	int s
	)
{
	return CalculateRectangularSum( row, column-s/2, s/2, s )-1*CalculateRectangularSum( row-s/2, column-s/2, s/2, s );
}


float CSURF::getAngle(
	float X,
	float Y
	)
{
	if( X>0 && Y>=0 )
	{
		return (float)( atan(Y/X) );
	}

	if( X<0 && Y>=0 )
	{
		return (float)( M_PI-atan(-Y/X) );
	}

	if( X<0 && Y<0 )
	{
		return (float)( M_PI+atan(Y/X) );
	}

	if( X>0 && Y<0 )
	{
		return (float)( 2*M_PI-atan(-Y/X) );
	}

	return 0;
}


inline float CSURF::gaussian(
	int x,
	int y,
	float sig
	)
{
	return (float)( (1.0f/(2.0f*M_PI*sig*sig))*exp(-(x*x+y*y)/(2.0f*sig*sig)) );
}


inline float CSURF::gaussian(
	float x,
	float y,
	float sig
	)
{
	return (float)( 1.0f/(2.0f*M_PI*sig*sig)*exp(-(x*x+y*y)/(2.0f*sig*sig)) );
}


void CSURF::SettingIniFact(
	int w,
	int h
	)
{
	m_nwidth = w;
	m_nheight = h;

	if( m_pIntegralImage!=NULL )
	{
		free( m_pIntegralImage );
		m_pIntegralImage = NULL;
	}
	m_pIntegralImage = (float*)malloc( sizeof(float)*w*h );
	memset( m_pIntegralImage, 0x00, sizeof(float)*m_nwidth*m_nheight );
}


void CSURF::nrerror(
        char error_text[]
        )
{
        fprintf(stderr,"Numerical Recipes run-time error...\n");
        fprintf(stderr,"%s\n",error_text);
        fprintf(stderr,"...now exiting to system...\n");
        exit(1);
}


float **CSURF::matrix_allocate(
        long nrl,
        long nrh,
        long ncl,
        long nch
        )
{
        long i, nrow=nrh-nrl+1, ncol=nch-ncl+1;
        float **m;

        /* allocate pointers to rows */
        m = (float **)malloc( (size_t)((nrow+NR_END)*sizeof(float*)) );
        char str1[]="allocation failure 1 in matrix()";
        if( !m )
                nrerror( str1 );
        m += NR_END;
        m -= nrl;

        /* allocate rows and set pointers to them */
        m[nrl] = (float *)malloc( (size_t)((nrow*ncol+NR_END)*sizeof(float)) );
        char str2[]="allocation failure 2 in matrix()";
        if( !m[nrl] )
                nrerror( str2 );
        m[nrl] += NR_END;
        m[nrl] -= ncl;

        for( i=nrl+1; i<=nrh; i++ )
        {
                m[i] = m[i-1]+ncol;
        }

        /* return pointer to array of pointers to rows */
        return m;
}


int *CSURF::ivector(
        long nl,
        long nh
        )
{
        int *v;

        v = (int *)malloc( (size_t)((nh-nl+1+NR_END)*sizeof(int)) );
        char str[]="allocation failure in ivector()";
        if( !v )
                nrerror( str );
        return v-nl+NR_END;
}


float *CSURF::vector_allocate(
        long nl,
        long nh
        )
{
        float *v;

        v = (float *)malloc( (size_t)((nh-nl+1+NR_END)*sizeof(float)) );
        char str[]="allocation failrue in vector()";
        if( !v )
                nrerror( str );
        return v-nl+NR_END;
}


void CSURF::free_matrix(
        float **m,
        long nrl,
        long nrh,
        long ncl,
        long nch
        )
{
        free((FREE_ARG) (m[nrl]+ncl-NR_END));
        free((FREE_ARG) (m+nrl-NR_END));
}


void CSURF::free_ivector(
        int *v,
        long nl,
        long nh
        )
{
        free((FREE_ARG) (v+nl-NR_END));
}


void CSURF::free_vector(
        float *v,
        long nl,
        long nh
        )
{
        free( (FREE_ARG)(v+nl-NR_END) );
}


// LU decomposition
void CSURF::ludcmp(
	float **a,
	int n,
	int *indx,
	float *d
	)
{
	register int i, imax, j, k;
	float big, dum, sum, temp;
	float *vv;

	vv = vector_allocate( 1,n );
	*d = 1.0;
	for( i=1; i<=n; i++ )
	{
		big = 0.f;
		for( j=1; j<=n; j++ )
		{
			if( (temp=fabs(a[i][j]))>big )
			{
				big = temp;
			}
		}
		if( big==0.0 )
		{
			char str[]="Singular matrix in routine ludcmp";
			nrerror( str );
		}
		vv[i] = 1.f/big;
	}
	//Below the loop over columns of Crout's method
	for( j=1; j<=n; j++ )
	{
		for( i=1; i<j; i++ )
		{
			sum = a[i][j];
			for( k=1; k<i; k++ )
			{
				sum -= a[i][k]*a[k][j];
			}
			a[i][j] = sum;
		}
		big = 0.f;
		for( i=j; i<=n; i++ )
		{
			sum = a[i][j];
			for( k=1; k<j; k++ )
			{
				sum -= a[i][k]*a[k][j];
			}
			a[i][j] = sum;
			if( (dum=vv[i]*fabs(sum))>=big )
			{
				big = dum;
				imax = i;
			}
		}
		if( j!=imax )
		{
			for( k=1; k<=n; k++ )
			{
				dum = a[imax][k];
				a[imax][k] = a[j][k];
				a[j][k] = dum;
			}
			*d = -(*d);
			vv[imax] = vv[j];
		}
		indx[j] = imax;
		if( a[j][j]==0.0 )
		{
			a[j][j] = (float)TINY;
		}
		if( j!=n )
		{
			dum = 1.f/(a[j][j]);
			for( i=j+1; i<=n; i++ )
			{
				a[i][j] *= dum;
			}
		}
	}
	free_vector( vv, 1, n );
}


void CSURF::lubksb(
	float **a,
	int n,
	int *indx,
	float *b
	)
{
	int i, ii = 0, ip, j;
	float sum;

	for( i=1; i<=n; i++ )
	{
		ip = indx[i];
		sum = b[ip];
		b[ip] = b[i];

		if( ii )
		{
			for( j=ii; j<=i-1; j++ )
			{
				sum -= a[i][j]*b[j];
			}
		}
		else if( sum )
		{
			ii = i;
		}

		b[i] = sum;
	}
	for( i=n; i>=1; i-- )
	{
		sum = b[i];
		for( j=i+1; j<=n; j++ )
		{
			sum -=a[i][j]*b[j];
		}
		b[i] = sum/a[i][i];
	}
}


void CSURF::Surf(
	sensor_msgs::Image image,
	std::vector<Interestpoint> &itpointTmp
	)
{
	int w=image.width, h=image.height;

	SettingIniFact( w, h );
	CalculateIntgral( image );
	ConstructScaleSpace();
	LocaliseInterestPoint( 0.0004f );
	GetInterestPoint( itpointTmp );
	getDescriptors( itpointTmp );
}
