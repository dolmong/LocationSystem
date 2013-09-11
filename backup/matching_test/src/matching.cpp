#include "matching.h"

#define TINY 1.0e-20
#define NR_END 1
#define FREE_ARG char*

CMatching::CMatching()
{
}

CMatching::~CMatching()
{
}


void CMatching::getMatches(
	std::vector<Interestpoint> &SavePrmt1,
	std::vector<Interestpoint> &SavePrmt2,
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec
	)
{
	int ChkSize1=SavePrmt1.size();
	int ChkSize2=SavePrmt2.size();

	IpPairVec.clear();

	ChkSize1<ChkSize2 ? getMatchesInlineFunc( SavePrmt1, SavePrmt2, IpPairVec, false ):getMatchesInlineFunc( SavePrmt2, SavePrmt1, IpPairVec, true );
}


void CMatching::getMatchesInlineFunc(
	std::vector<Interestpoint> &Prmt1, 
	std::vector<Interestpoint> &Prmt2, 
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec,
	bool chkchg
	)
{
	float dist=0.f, d1=0.f, d2=0.f;
	Interestpoint *match;

	float sum=0.f, diff=0.f;
	for( unsigned int i=0; i<Prmt1.size(); i++ )
	{
		d1 = d2 = FLT_MAX;


		for( unsigned int j=0; j<Prmt2.size(); j++ )
		{
			sum = 0.f;
			diff = 0.f;
			Interestpoint *p_ref = (Interestpoint *)&Prmt1[i];
			Interestpoint *p_input = (Interestpoint *)&Prmt2[j];

			if( p_ref->_Lap==p_input->_Lap )
			{
				for( int k=0; k<64; ++k )
				{
					diff = p_ref->_IpDescriptor[k]-p_input->_IpDescriptor[k];
					sum += diff*diff;
				}
				dist = sqrt(sum);

				if( dist<d1 )
				{
					d2 = d1;
					d1 = dist;
					match = &Prmt2[j];
				}
				else if( dist<d2 )
					d2 = dist;
			}
		}

		if( d1/d2<0.5 )
		{
			if( chkchg!=true )
			{
				Prmt1[i]._dx = match->_x-Prmt1[i]._x;
				Prmt1[i]._dy = match->_y-Prmt1[i]._y;
				match->_chkOverlap++;
				IpPairVec.push_back( std::make_pair(Prmt1[i], *match) );
			}
			else
			{
				match->_dx = Prmt1[i]._x-match->_x; 
				match->_dy = Prmt1[i]._y-match->_y;
				match->_chkOverlap += 1;
				IpPairVec.push_back( std::make_pair(*match, Prmt1[i]) );				
			}
		}
	}
}


void CMatching::erasePt(
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec,
	std::vector<int> &PtIndx
	)
{
	std::vector<std::pair<Interestpoint, Interestpoint> >::iterator iter;

	int tempIndx=0;
	for( int i=PtIndx.size()-1; i>=0; i-- )
	{
		tempIndx = PtIndx[i];
		iter = IpPairVec.begin()+tempIndx;
		IpPairVec.erase( iter );
	}
}


void CMatching::eraseFarOutlier(
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec
	)
{
	int SumCount=0;
	float SumRefX=0.f, SumRefY=0.f, SumInptX=0.f, SumInptY=0.f;
	float AvgRefX=0.f, AvgRefY=0.f, AvgInptX=0.f, AvgInptY=0.f;
	float RefTmpDist=0.f, InptTmpDist=0.f;
	float SumRefAvgToPtDist=0.f, SumInptAvgToPtDist=0.f;
	float TempRefX=0.f, TempRefY=0.f, TempInptX=0.f, TempInptY=0.f;
	float AvgRefDist=0.f, AvgInptDist=0.f;
	int AvgDistRefCount=0, AvgDistInptCount=0;

	std::vector<AvgDist> AvgDistContainer;
	std::vector<float> AvgToPtDistRefInpt;
	std::vector<int> AvgDistIndx;
	
	for( int j=0; j<5; j++ )
	{		
		SumCount = IpPairVec.size();
		SumRefX = 0.f, SumRefY = 0.f, SumInptX = 0.f, SumInptY = 0.f;
		for( int i=0; i<SumCount; i++ )
		{
			SumRefX += IpPairVec[i].first._vx;		SumRefY += IpPairVec[i].first._vy;
			SumInptX += IpPairVec[i].second._vx;	SumInptY += IpPairVec[i].second._vy;
		}
		AvgRefX = SumRefX/SumCount, AvgRefY = SumRefY/SumCount;
		AvgInptX = SumInptX/SumCount, AvgInptY = SumInptY/SumCount;

		RefTmpDist = 0.f, InptTmpDist = 0.f;
		SumRefAvgToPtDist = 0.f, SumInptAvgToPtDist = 0.f;
		TempRefX = 0.f, TempRefY = 0.f, TempInptX = 0.f, TempInptY = 0.f;
		for( int i=0; i<SumCount; i++ )
		{
			TempRefX = IpPairVec[i].first._vx, TempRefY = IpPairVec[i].first._vy;
			TempInptX = IpPairVec[i].second._vx, TempInptY = IpPairVec[i].second._vy;			

			RefTmpDist = sqrt(((AvgRefX-TempRefX)*(AvgRefX-TempRefX))+((AvgRefY-TempRefY)*(AvgRefY-TempRefY)));
			InptTmpDist = sqrt(((AvgInptX-TempInptX)*(AvgInptX-TempInptX))+((AvgInptY-TempInptY)*(AvgInptY-TempInptY)));

			SumRefAvgToPtDist += RefTmpDist;
			SumInptAvgToPtDist += InptTmpDist;
		}

		AvgRefDist = SumRefAvgToPtDist/SumCount, AvgInptDist = SumInptAvgToPtDist/SumCount;
		AvgDistRefCount = 0, AvgDistInptCount = 0;
		for( int i=0; i<SumCount; i++ )
		{
			TempRefX = IpPairVec[i].first._vx, TempRefY = IpPairVec[i].first._vy;
			TempInptX = IpPairVec[i].second._vx, TempInptY = IpPairVec[i].second._vy;

			if( pow((TempRefX-AvgRefX), 2)+pow((TempRefY-AvgRefY), 2)>pow(AvgRefDist, 2) )
				AvgDistRefCount++;

			if( pow((TempInptX-AvgInptX), 2)+pow((TempInptY-AvgInptY), 2)>pow(AvgInptDist, 2) )
				AvgDistInptCount++;
		}

		AvgDist AvgToPtDist;
		AvgDistContainer.clear();
		for( int i=0; i<SumCount; i++ )
		{
			TempRefX = IpPairVec[i].first._vx, TempRefY = IpPairVec[i].first._vy;
			TempInptX = IpPairVec[i].second._vx, TempInptY = IpPairVec[i].second._vy;

			AvgToPtDist.AvgRefToPtDist = sqrt((pow((AvgRefX-TempRefX), 2))+(pow((AvgRefY-TempRefY), 2)))/AvgRefDist;
			AvgToPtDist.AvgInpToPttDist = sqrt((pow((AvgInptX-TempInptX), 2))+(pow((AvgInptY-TempInptY), 2)))/AvgInptDist;
			AvgDistContainer.push_back(AvgToPtDist);
		}
		AvgToPtDistRefInpt.clear();
		AvgDistIndx.clear();

		for( unsigned int i=0; i<AvgDistContainer.size(); i++ )
		{
			float TempThisFor = abs(AvgDistContainer[i].AvgRefToPtDist+AvgToPtDist.AvgInpToPttDist)/2;
			AvgToPtDistRefInpt.push_back( TempThisFor );

			if( TempThisFor>=5.f )
				AvgDistIndx.push_back( i );
		}		
		erasePt( IpPairVec,AvgDistIndx );

		AvgToPtDistRefInpt.clear();
	}
}

void CMatching::eraseMissMatchesUsingMean(
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec,
	sensor_msgs::Image *imgRef,
	sensor_msgs::Image *imgIn
	)
{
	int MaskSize1=0, MaskSize2=0;
	float Mean1=0.f, Mean2=0.f;
	std::vector<int> Indx;
	for( int i=0; i<IpPairVec.size(); i++ )
	{
		MaskSize1 = chkScale(IpPairVec[i].first._scale);
		MaskSize2 = chkScale(IpPairVec[i].second._scale);

		Mean1 = calculMean( (int)IpPairVec[i].first._vx, (int)IpPairVec[i].first._vy, MaskSize1, imgRef->width, imgRef->height, &(imgRef->data) );
		Mean2 = calculMean( (int)IpPairVec[i].second._vx, (int)IpPairVec[i].second._vy, MaskSize2, imgIn->width, imgIn->height, &(imgIn->data) );

		if( abs(Mean1-Mean2)>30 )
			Indx.push_back(i);
	}
	erasePt( IpPairVec, Indx );
	Indx.clear();
}


void CMatching::EraseMissDis(
	std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching,
	float Desired
	)
{
	std::vector<std::pair<Interestpoint, Interestpoint> >::iterator iter;
	float rx=0.f, ry=0.f, ix=0.f, iy=0.f;
	float fDistmp=0.f;
	int nErase=0;

	int nNum = ipMatching.size();
	for( int i=0; i<nNum; i++ )
	{
		rx = ipMatching[i].first._vx;	ry = ipMatching[i].first._vy;
		ix = ipMatching[i].second._vx;	iy = ipMatching[i].second._vy;
		fDistmp = ipMatching[i].first._dis = ipMatching[i].second._dis = sqrt( pow(rx-ix, 2)+pow(ry-iy, 2) );

		if( fDistmp>Desired+0.2 || fDistmp<Desired-0.2 )
		{
			int Indx=i;
			i--;
			nNum--;
			iter = ipMatching.begin()+Indx;
			ipMatching.erase( iter );
			nErase++;
		}
	}
}


int CMatching::chkScale(
	float scale
	)
{
	int maskSize=0;

	if( scale>6.5 )
		maskSize = 6;
	else if( scale <= 6.5 && scale>4.5 )
		maskSize = 5;
	else if( scale <= 4.5 && scale>2.5 )
		maskSize = 4;
	else if( scale <= 2.5 && scale>1.5 )
		maskSize = 3;
	else if( scale <= 1.5 && scale>=0 )
		maskSize = 2;

	return maskSize;
}


float CMatching::calculMean(
	int x,
	int y,
	int masksize,
	int width,
	int height,
	std::vector<unsigned char> *data
	)
{
	ROS_INFO( "image size: [%d]", data->size() );
	for( unsigned int i=0; i<data->size(); i++ )
	{
		
	}

	float meanVal=0.f;
	for( int j=y-masksize; j<y+masksize+1; j++ )
	{
		for( int i=x-masksize; i<x+masksize+1; i++ )
		{
			if( (x-masksize<0) || (y-masksize<0) || (x+masksize>width) || (y+masksize>height) )
				meanVal += 0;
			else
				meanVal += data->at(j*width+i);			
		}
		meanVal = meanVal/((2*masksize+1)*(2*masksize+1));
	}

	return meanVal;
}


void CMatching::SetVector(
	std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching
	)
{
	float rx=0.f, ry=0.f, ix=0.f, iy=0.f;
	register float fFocallenX=931.3732f, fFocallenY=927.5794f;
	float fHeight=0.f;

	for( int i=0; i<ipMatching.size(); i++ )
	{
		fHeight = ipMatching[i].second._height = ipMatching[i].first._height;
		
		rx = ipMatching[i].first._vx = fHeight*ipMatching[i].first._x/fFocallenX;
		ry = ipMatching[i].first._vy = fHeight*ipMatching[i].first._y/fFocallenY;
		ix = ipMatching[i].second._vx = fHeight*ipMatching[i].second._x/fFocallenX;
		iy = ipMatching[i].second._vy = fHeight*ipMatching[i].second._y/fFocallenY;

		ipMatching[i].second._dis = ipMatching[i].first._dis = sqrt( pow(rx-ix, 2)+pow(ry-iy, 2) );
	}
}


bool CMatching::SetHeight(
	std::string *strFile,
	std::vector<Interestpoint> &itpointTmp
	)
{
	CFileSystem filesystem;

	sensor_msgs::Image imgHeight;

	if( !filesystem.ReadImage( imgHeight, strFile->c_str() ) )
	{
		ROS_ERROR( "Don't read HeightImage!" );
		return false;
	}
	else
	{
		int x=0, y=0;
		for( unsigned int i=0; i<itpointTmp.size(); i++ )
		{
			x = (int)( itpointTmp[i]._x+0.5 );
			y = (int)( itpointTmp[i]._y+0.5 );
			itpointTmp[i]._height = imgHeight.data.at( y*imgHeight.width+x );
		}
	}

	return true;
}


void CMatching::GetSixParameter(
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVec,
	SixPrmt &spTmp
	)
{
	int count_Mtch=IpPairVec.size();
	int count_MtchErr=0;

	std::vector<std::pair<Interestpoint, Interestpoint> > IpPairVecS;

	std::vector<std::pair<Interestpoint, Interestpoint> >::iterator iter;

	for( register int i=0; i<count_Mtch; i++)
	{
		IpPairVecS.push_back( IpPairVec[i] );
	}

	ErrValue ErrV;
	int Indx_max=0;	
	float ErrMax=0;

	for( register int j=0; j<count_Mtch; j++)
	{
		DoLeastSquare( IpPairVecS, spTmp );

		count_MtchErr = IpPairVecS.size();

		if( count_MtchErr<=3 )
			break;

		ErrMax = 0;

		float rx=0.f, ry=0.f, ix=0.f, iy=0.f;
		float tmpix=0.f, tmpiy=0.f;
		for( register int i=0; i<count_MtchErr; i++)
		{
			float RefX = 0, RefY = 0, InX = 0, InY = 0;
			float TempInX = 0, TempInY = 0;	

			rx = IpPairVecS[i].first._vx;
			ry = IpPairVecS[i].first._vy;
			ix = IpPairVecS[i].second._vx;
			iy = IpPairVecS[i].second._vy;

			tmpix = spTmp._A*RefX+spTmp._B*RefY+spTmp._C;
			tmpiy = spTmp._D*RefX+spTmp._E*RefY+spTmp._F;

			ErrV.ErrX = abs( tmpix-ix );
			ErrV.ErrY = abs( tmpiy-iy );

			if( (ErrV.ErrX+ErrV.ErrY)>ErrMax )
			{
				ErrMax = ErrV.ErrX+ErrV.ErrY;
				Indx_max = i;
			}
		}
		ErrMax = ErrMax/2;
		if( ErrMax<0.01 )
			break;

		iter = IpPairVecS.begin()+Indx_max;
		IpPairVecS.erase( iter );
	}

	DoLeastSquare( IpPairVecS, spTmp );

	ROS_INFO( "Error: [%3.3f]", ErrMax );
	ROS_INFO( "Final Matching Count: [%3d]", count_MtchErr );
	ROS_INFO( "a: [%3.3f]\tb: [%3.3f]\tc: [%3.3f]", spTmp._A, spTmp._B, spTmp._C );
	ROS_INFO( "d: [%3.3f]\te: [%3.3f]\tf: [%3.3f]", spTmp._D, spTmp._E, spTmp._F );

	IpPairVec.clear();
	for( unsigned int i=0; i<IpPairVecS.size(); i++ )
	{
		IpPairVec.push_back( IpPairVecS[i] );
	}
}


void CMatching::DoLeastSquare(
	std::vector<std::pair<Interestpoint, Interestpoint> > &IpPairVecL,
	SixPrmt &spTmp
	)
{
	float SumRefx2=0.f, SumRefxy=0.f, SumRefy2=0.f, SumRefx=0.f, SumRefy=0.f;
	float SumRefxInx=0.f, SumRefyInx=0.f, SumInx=0.f, SumRefxIny=0.f, SumRefyIny=0.f, SumIny=0.f;
	float CalculTempRefX=0.f, CalculTempRefY=0.f, CalculTempInX=0.f ,CalculTempInY=0.f;
	int count_Mtch=IpPairVecL.size();

	for( int i=0; i<count_Mtch; i++ )
	{
		CalculTempRefX = IpPairVecL[i].first._vx;
		CalculTempRefY = IpPairVecL[i].first._vy;
		CalculTempInX = IpPairVecL[i].second._vx;
		CalculTempInY = IpPairVecL[i].second._vy;

		SumRefx += CalculTempRefX;
		SumRefx2 += pow( CalculTempRefX, 2 );
		SumRefy += CalculTempRefY;
		SumRefy2 += pow( CalculTempRefY, 2 );
		SumRefxy += CalculTempRefX*CalculTempRefY;

		SumRefxInx += CalculTempRefX*CalculTempInX;
		SumRefyInx += CalculTempRefY*CalculTempInX;
		SumInx += CalculTempInX;

		SumRefxIny += CalculTempRefX*CalculTempInY;
		SumRefyIny += CalculTempRefY*CalculTempInY;
		SumIny += CalculTempInY;
	}

	float **a=NULL, *b=NULL, d=0.f;
	int n=3, *indx=NULL;


	a = matrix_allocate( 1, 3, 1, 3 );
	b = vector_allocate( 1, 3 );
	indx = ivector( 1, 3 );

	a[1][1] = SumRefx2;			a[1][2] = SumRefxy;			a[1][3] = SumRefx;
	a[2][1] = SumRefxy;			a[2][2] = SumRefy2;			a[2][3] = SumRefy;			
	a[3][1] = SumRefx;			a[3][2] = SumRefy;			a[3][3] = (float)count_Mtch;
	b[1] = SumRefxInx;			b[2] = SumRefyInx;			b[3] = SumInx;
	ludcmp( a, n, indx, &d );
	lubksb( a, n, indx, b );
	spTmp._A = b[1];			spTmp._B = b[2];			spTmp._C = b[3];

	a[1][1] = SumRefx2;			a[1][2] = SumRefxy;	 		a[1][3] = SumRefx;
	a[2][1] = SumRefxy;			a[2][2] = SumRefy2;			a[2][3] = SumRefy;			
	a[3][1] = SumRefx;			a[3][2] = SumRefy;			a[3][3] = (float)count_Mtch;
	b[1] = SumRefxIny;			b[2] = SumRefyIny;			b[3] = SumIny;
	ludcmp( a, n, indx, &d );
	lubksb( a, n, indx, b );
	spTmp._D = b[1];			spTmp._E = b[2];			spTmp._F = b[3];

	free_matrix( a, 1, 3, 1, 3 );
	free_vector( b, 1, 3 );
	free_ivector( indx, 1, 3 );
}


void CMatching::nrerror(
        char error_text[]
        )
{
        fprintf(stderr,"Numerical Recipes run-time error...\n");
        fprintf(stderr,"%s\n",error_text);
        fprintf(stderr,"...now exiting to system...\n");
        exit(1);
}


float **CMatching::matrix_allocate(
        long nrl,
        long nrh,
        long ncl,
        long nch
        )
{
        long i, nrow=nrh-nrl+1, ncol=nch-ncl+1;
        float **m;

        m = (float **)malloc( (size_t)((nrow+NR_END)*sizeof(float*)) );
        char str1[]="allocation failure 1 in matrix()";
        if( !m )
                nrerror( str1 );
        m += NR_END;
        m -= nrl;

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

        return m;
}


int *CMatching::ivector(
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


float *CMatching::vector_allocate(
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


void CMatching::free_matrix(
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


void CMatching::free_ivector(
        int *v,
        long nl,
        long nh
        )
{
        free((FREE_ARG) (v+nl-NR_END));
}


void CMatching::free_vector(
        float *v,
        long nl,
        long nh
        )
{
        free( (FREE_ARG)(v+nl-NR_END) );
}


void CMatching::ludcmp(
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


void CMatching::lubksb(
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


void CMatching::Matching(
	sensor_msgs::Image *imgRef,
	sensor_msgs::Image *imgIn,
	std::vector<Interestpoint> *ipRef,
	std::vector<Interestpoint> *ipIn,
	std::string *strHeight,
	std::vector<std::pair<Interestpoint, Interestpoint> > &ipMatching,
	SixPrmt	&spTmp
	)
{
	if( SetHeight( strHeight, *ipRef ) )
	{
		getMatches( *ipRef, *ipIn, ipMatching );
		SetVector( ipMatching );
		eraseFarOutlier( ipMatching );
		eraseMissMatchesUsingMean( ipMatching, imgRef, imgIn );

		if( ipMatching.size()>=3 )
			GetSixParameter( ipMatching, spTmp );
		else if( ipMatching.size()==0 )
			ROS_INFO( "less than 3 matching." );
	}
	else
		return;
}
