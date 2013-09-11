#ifndef LOCATIONSYSTEM_PROJECT_SURF_H
#define LOCATIONSYSTEM_PROJECT_SURF_H

#include <malloc.h>
#include <vector>
#include <math.h>
#include <float.h>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
//#include "locationsystem/Matrix.h"

#define Octave_Num 5
#define Layer_Num  4

#define TINY 1.0e-20

#define NR_END 1
#define FREE_ARG char*

class CSURF
{
public:
	// Fucntion
	CSURF(void);
	~CSURF(void);

private:
	// Structure
	typedef struct _ResponseLayer
	{
		int	_width;
		int	_height;
		int	_interval;
		int	_filter;	
		float	*_ptrace;  // size = _width*_height
		float	*_pdetH;   // size = _width*_height
	} Responselayer;
	
	typedef struct _InterestPoint 
	{
		float	_x;
		float	_y;
		float	_scale;
		int	_Lap;
		float	_majordiretion;
		float	_IpDescriptor[64];
		float	_dx;
		float	_dy;
		int	_filter;
		int	_chkOverlap;
		float	_height;
		float	_vx;
		float	_vy;
		float	_dis;
	} Interestpoint;

private:
	// Variable
	float *m_pIntegralImage;
	int m_nwidth;
	int m_nheight;
	Responselayer m_layer[Octave_Num][Layer_Num];
	int des_index;
	float m_fthreshold;
	std::vector<Interestpoint> m_vInterestpoint;

private:
	// Inline Function
	inline int fRound( float flt );
	inline float haarX(int row, int column, int size);
	inline float haarY(int row, int column, int size);
	inline float gaussian(int x, int y, float sig);
	inline float gaussian(float x, float y, float sig);

private:
	// Function
	void SettingIniFact( int width, int height );
	void CalculateIntgral( sensor_msgs::Image image );
	void ConstructScaleSpace();
	void LocaliseInterestPoint( float threshold=0.f );
	void GetInterestPoint( std::vector<Interestpoint> &vItpoint );
	void getDescriptors( std::vector<Interestpoint> &vItpoint );
	void getOrientation( std::vector<Interestpoint> &vItpoint );
	void getDescriptor( std::vector<Interestpoint> &vItpoint );
	float getAngle(float X, float Y);
	void nrerror( char error_text[] );
	float **matrix_allocate( long nrl, long nrh, long ncl, long nch );
	int *ivector( long nl, long nh );
	float *vector_allocate( long nl, long nh );
	void free_matrix( float **m, long nrl, long nrh, long ncl, long nch );
	void free_ivector( int *v, long nl, long nh );
	void free_vector( float *v, long nl, long nh );
	void ludcmp( float **a, int n, int *indx, float *d );
	void lubksb( float **a, int n, int *indx, float *b );
	float CalculateRectangularSum( int y, int x, int cy, int cx );
	void CalulateFastHessian( int octave, int layer );
	bool ApplyNonMaximalSuppression( int octave, int layer, int y, int x );
	void InterpolateInterestpoint( int octave, int layer, int x, int y );

public:
	// Variable
	std::vector<Interestpoint> ipSurf;

public:
	// Function
	void Surf( sensor_msgs::Image image, std::vector<Interestpoint> &itpointTmp );
};

#endif
