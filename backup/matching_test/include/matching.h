#ifndef MATCHING_CLASS_MATCHING_H
#define MATCHING_CLASS_MATCHING_H

#include <malloc.h>
#include <vector>
#include <utility>
#include <math.h>
#include <float.h>
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "global.h"
#include "filesystem.h"


class CMatching
{
public:
	// Constructor, Destructor
	CMatching(void);
	~CMatching(void);

private:
	// Structural
	typedef struct _SixPrmt
	{
		float _A, _B, _C, _D, _E, _F;
	} SixPrmt;

	typedef struct _ErrValue
	{
		float ErrX, ErrY;
	} ErrValue;

	typedef struct _AvgDist 
	{
		float AvgRefToPtDist;
		float AvgInpToPttDist;
	} AvgDist;

private:
	// Function
	void getMatches( std::vector<Interestpoint> &SavePrmt1, std::vector<Interestpoint> &SavePrmt2, std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec );
	void getMatchesInlineFunc( std::vector<Interestpoint> &Prmt1, std::vector<Interestpoint> &Prmt2, std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec, bool chkchg );
	void erasePt( std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec, std::vector<int> &PtIndx );
	int chkScale( float scale );
	float calculMean( int x, int y, int masksize, int width, int height, std::vector<unsigned char> *data );
	void SetVector( std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching );
	bool SetHeight( std::string *strPath, std::vector<Interestpoint> &itpointTmp );
	void eraseFarOutlier( std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec);
	void eraseMissMatchesUsingMean( std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec, sensor_msgs::Image *imgRef, sensor_msgs::Image *imgIn );
	void EraseMissDis( std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching, float Desired );
	void GetSixParameter( std::vector<std::pair<Interestpoint, Interestpoint> > &IpPaitrVec, SixPrmt &spTmp );
	void DoLeastSquare( std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVecL, SixPrmt &spTmp );

private:
	// Matrix Function
	void nrerror( char error_text[] );
	float **matrix_allocate( long nrl, long nrh, long ncl, long nch );
	int *ivector( long nl, long nh );
	float *vector_allocate( long nl, long nh );
	void free_matrix( float **m, long nrl, long nrh, long ncl, long nch );
	void free_ivector( int *v, long nl, long nh );
	void free_vector( float *v, long nl, long nh );
	void ludcmp( float **a, int n, int *indx, float *d );
	void lubksb( float **a, int n, int *indx, float *b );

public:
	// Function
	void Matching( sensor_msgs::Image *imgRef, sensor_msgs::Image *imgIn, std::vector<Interestpoint> *ipRef, std::vector<Interestpoint> *ipIn, std::string *strRef, std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching, SixPrmt &spTmp );

public:
	// Variable
	std::vector<std::pair<Interestpoint, Interestpoint> > ipPairMat;
	SixPrmt spMeter;
};

#endif
