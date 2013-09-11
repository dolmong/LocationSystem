#include "SURF.h"

// 추가 헤더파일
#include <float.h>
#include <math.h>
#include <vector>
#include "Matrix.h"
#include "RGBBYTE.h"
#include "StopWatch.h"
#include "MainFrm.h"

using namespace std;


#define M_PI 3.14159265358979323846
#define TINY 1.0e-20

extern HWND hSurfWnd;

// construct
CMatching::CMatching(void)
{
}

// destroy
CMatching::~CMatching(void)
{

}


void CMatching::getMatches(
	std::vector<Interestpoint> &SavePrmt1,
	std::vector<Interestpoint> &SavePrmt2,
	std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec
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
	std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec,
	bool chkchg
	)
{
	float dist=0.f, d1=0.f, d2=0.f;
	Interestpoint *match;

	int count_Save=0, count_cc=0;
	float sum=0.f, diff=0.f;
	for( unsigned int i=0; i<Prmt1.size(); i++ )
	{
		d1 = d2 = FLT_MAX;
		cout_Save = Prmt1.size();
		cout_cc = 0;


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

				if( dist<d1 ) // if this feature matches better than current best
				{
					d2 = d1;
					d1 = dist;
					match = &Prmt2[j];
				}
				else if( dist<d2 ) // this feature matches better than second best
					d2 = dist;
			}
		}

		// If match has a d1:d2 ratio < 0.65 ipoints are a match
		// Change for getting more Interest point
		if( d1/d2<0.5 )
		{
			// Store the change in position
			if( chkchg!=TRUE )
			{
				Prmt1[i]._dx = match->_x-Prmt1[i]._x;
				Prmt1[i]._dy = match->_y-Prmt1[i]._y;
				match->_chkOverlap++;
				IpPairVec.push_back( make_pair(Prmt1[i], *match) );
			}
			else
			{
				match->_dx = Prmt1[i]._x-match->_x; 
				match->_dy = Prmt1[i]._y-match->_y;
				match->_chkOverlap += 1;
				IpPairVec.push_back( make_pair(*match, Prmt1[i]) );				
			}
		}
	}
}


void CMatching::erasePt(
	std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec,
	std::vector<int> &PtIndx
	)
{
	std::vector<pair<Interestpoint, Interestpoint>>::iterator iter;
	int countAvg=PtIndx.size();

	int tempIndx=0;
	for( int i=countAvg-1; i>=0; i-- )
	{
		tempIndx = PtIndx[i];
		iter = IpPairVec.begin()+tempIndx;
		IpPairVec.erase( iter );
	}
}


void CMatching::eraseFarOutlier(
	std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec
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

		//원의 방정식을 이용하여 무게 중심으로 부터 평균 거리 만큼 떨어진 걸이에
		//몇 개의 점들이 들어 있는지 확인.
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
		/////////////////////////////////////////////////////////////////////////

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

		
		AvgDistContainer.clear();
		AvgToPtDistRefInpt.clear();
		AvgDistIndx.clear();
	}
}

void CMatching::eraseMissMatchesUsingMean(
	std:: vector<pair<Interestpoint, Interestpoint>> &IpPairVec,
	CDib& dibRef,
	CDib& dibIn
	)
{
	int countMean=IpPairVec.size();
	float Mean1=0.f, Mean2=0.f;
	std::vector<int> Indx;
	Indx.clear();

	CDib dibCpyRef = dibRef;
	CDib dibCpyIn = dibIn;

	register int nRefW = dibCpyRef.GetWidth(), nRefH = dibCpyRef.GetHeight(), nRefC = dibCpyRef.GetBitCount();
	register int nInW = dibCpyIn.GetWidth(), nInH = dibCpyIn.GetHeight(), nInC = dibCpyIn.GetBitCount();

	if( nRefC!=8 )
		Grayscale( dibCpyRef );
	if( nInC!=8 )
		Grayscale( dibCpyIn );

	BYTE** RefPtr = dibCpyRef.GetPtr();
	BYTE** InPtr = dibCpyIn.GetPtr();

	for( int i=0; i < countMean; i++ )
	{
		int MaskSize1, MaskSize2;
		MaskSize1 = chkScale(IpPairVec[i].first._scale);
		MaskSize2 = chkScale(IpPairVec[i].second._scale);

		Mean1 = calculMean(
			(int)IpPairVec[i].first._vx, (int)IpPairVec[i].first._vy, MaskSize1, nRefW, nRefH, RefPtr
			);
		Mean2 = calculMean(
			(int)IpPairVec[i].second._vx, (int)IpPairVec[i].second._vy, MaskSize2, nInW, nInH, InPtr
			);

		if( abs(Mean1-Mean2)>30 )
			Indx.push_back(i);
	}
	erasePt( IpPairVec, Indx );
}


void CMatching::EraseMissDis(
	vector<pair<Interestpoint, Interestpoint>>& ipMatching,
	float Desired
	)
{
	vector<pair<Interestpoint, Interestpoint>>::iterator iter;
	float RefX = 0.f, RefY = 0.f, InX = 0.f, InY = 0.f;
	float fDistmp = 0.f;
	int nErase = 0;

	int nNum = ipMatching.size();
	printf( "매칭 수 : %d\n", nNum );
	for( int i=0; i<nNum; i++ )
	{
		RefX = ipMatching[i].first._vx;		RefY = ipMatching[i].first._vy;
		InX = ipMatching[i].second._vx;		InY = ipMatching[i].second._vy;
		fDistmp = ipMatching[i].first._dis = ipMatching[i].second._dis = sqrt( pow(RefX-InX, 2)+pow(RefY-InY, 2) );

		if( fDistmp>Desired+0.2 || fDistmp<Desired-0.2 )
		{
			int Indx = i;
			i--;	nNum--;
			iter = ipMatching.begin() + Indx;
			ipMatching.erase( iter );
			nErase++;
		}
	}
	printf( "제거된 매칭 수 : %d\n", nErase );
}


int CMatching::chkScale(
	float scale
	)
{
	int maskSize = 0;

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
	BYTE** ptr
	)
{
	float meanVal = 0.f;
	for( int j=y-masksize; j<y+masksize+1; j++ )
	{
		for( int i=x-masksize; i<x+masksize+1; i++ )
		{
			if( (x-masksize<0) || (y-masksize<0) || (x+masksize>width) || (y+masksize>height) )
				meanVal += 0;
			else
				meanVal += ptr[j][i];			
		}
		meanVal = meanVal/((2*masksize+1)*(2*masksize+1));
	}

	return meanVal;
}


void CMatching::SetVector(
		vector<pair<Interestpoint, Interestpoint>>& ipMatching
		)
{
	int nNum = ipMatching.size();
	float RefX = 0.f, RefY = 0.f, InX = 0.f, InY = 0.f;
	register float fFocallenX = (float)931.3732, fFocallenY = (float)927.5794;
	// register float fFocallenX = (float)3200, fFocallenY = (float)2400;
	float fHeight = 0.f;
	for( int i=0; i<nNum; i++ )
	{
		fHeight = ipMatching[i].second._height = ipMatching[i].first._height;
		
		RefX = ipMatching[i].first._vx = fHeight*ipMatching[i].first._x/fFocallenX; // 기준영상
		RefY = ipMatching[i].first._vy = fHeight*ipMatching[i].first._y/fFocallenY;
		InX = ipMatching[i].second._vx = fHeight*ipMatching[i].second._x/fFocallenX; // 입력영상
		InY = ipMatching[i].second._vy = fHeight*ipMatching[i].second._y/fFocallenY;

		ipMatching[i].second._dis = ipMatching[i].first._dis = sqrt( pow(RefX-InX, 2)+pow(RefY-InY, 2) );

	}
}


BOOL CMatching::SetHeight(
	CString strPath,
	vector<Interestpoint> &itpointTmp
	)
{
	CString strHeight = strPath.Left( strPath.GetLength() - 4 ) + _T("_height.bmp");
	
	CDib dibHeight;
	dibHeight.Load( strPath );

	if( dibHeight.IsValid() )
	{
		BYTE** ptrHeight = dibHeight.GetPtr();

		UINT num = itpointTmp.size();
		int x, y;
		for( UINT i=0; i<num; i++ )
		{
			x = (int)( itpointTmp[i]._x+0.5 );
			y = (int)( itpointTmp[i]._y+0.5 );
			itpointTmp[i]._height = ptrHeight[y][x];
		}

		return TRUE;
	}

	return FALSE;
}


void CMatching::GetSixParameter(
	vector<pair<Interestpoint, Interestpoint>>& IpPairVec,
	SixPrmt& spTmp
	)
{
	int count_Mtch = IpPairVec.size();
	int count_MtchErr;

	vector<pair<Interestpoint, Interestpoint>> IpPairVecS;

	// vector형의 Data 제거를 돕기 위하여 iter를 선언
	vector<pair<Interestpoint, Interestpoint>>::iterator iter;

	for( register int i=0; i<count_Mtch; i++)
		IpPairVecS.push_back( IpPairVec[i] );

	ErrValue ErrV;
	int Indx_max = 0;	
	float ErrMax = 0;

	printf( "처음 정합점 개수: [%3d]\n", count_Mtch );

	for( register int j=0; j<count_Mtch; j++)
	{
		DoLeastSquare_Affine( IpPairVecS, spTmp );

		m_ErrValue.clear();
		count_MtchErr = IpPairVecS.size();

		if( count_MtchErr<=3 )
			break;

		ErrMax = 0;
		for( register int i=0; i<count_MtchErr; i++)
		{
			float RefX = 0, RefY = 0, InX = 0, InY = 0;
			float TempInX = 0, TempInY = 0;	
			float ErrX2 = 0, ErrY2 = 0;

			RefX = IpPairVecS[i].first._vx;
			RefY = IpPairVecS[i].first._vy;
			InX = IpPairVecS[i].second._vx;
			InY = IpPairVecS[i].second._vy;

			TempInX = spTmp._A*RefX+spTmp._B*RefY+spTmp._C;
			TempInY = spTmp._D*RefX+spTmp._E*RefY+spTmp._F;

			ErrV.ErrX = abs( TempInX -InX );
			ErrV.ErrY = abs( TempInY -InY );

			if( (ErrV.ErrX+ErrV.ErrY)>ErrMax )
			{
				ErrMax = ErrV.ErrX + ErrV.ErrY;
				Indx_max = i;
			}
			m_ErrValue.push_back( ErrV );
		}
		ErrMax = ErrMax/2;
		if( ErrMax<0.01 )
			break;

		//vector형의 원하는 부분의 Data제거
		iter = IpPairVecS.begin() + Indx_max;
		IpPairVecS.erase( iter );
	}

	DoLeastSquare_Affine( IpPairVecS, spTmp );	// affine parameter를 구함

	printf( "Error : [%3.3f]\n", ErrMax );
	printf( "최종 정합점 개수: [%3d]\n", count_MtchErr );
	printf( "a: [%3.5f],\tb: [%3.5f],\tc: [%3.5f]\nd: [%3.5f],\te: [%3.5f],\tf: [%3.5f]\n", 
		spTmp._A, spTmp._B, spTmp._C, spTmp._D, spTmp._E, spTmp._F );

	IpPairVec.clear();
	for( unsigned int i=0; i<IpPairVecS.size(); i++)	
		IpPairVec.push_back( IpPairVecS[i] );
}


void CMatching::DoLeastSquare_Affine(
	vector<pair<Interestpoint, Interestpoint>>& IpPairVecL,
	SixPrmt& spTmp
	)
{

	float SumRefx2 = 0.f, SumRefxy = 0.f, SumRefy2 = 0.f, SumRefx = 0.f, SumRefy = 0.f;
	float SumRefxInx = 0.f, SumRefyInx = 0.f, SumInx = 0.f, SumRefxIny = 0.f, SumRefyIny = 0.f, SumIny = 0.f;
	float CalculTempRefX = 0.f, CalculTempRefY = 0.f, CalculTempInX = 0.f ,CalculTempInY = 0.f;
	int count_Mtch = IpPairVecL.size();

	for( int i=0; i<count_Mtch; i++ )
	{
		CalculTempRefX = IpPairVecL[i].first._vx; // 기준영상
		CalculTempRefY = IpPairVecL[i].first._vy;
		CalculTempInX = IpPairVecL[i].second._vx; // 입력영상
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
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float **a = NULL, *b = NULL, d = 0.f;
	int n = 3, *indx = NULL;


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


void CMatching::Matching(
	CString	strRefPath,
	CString	strInPath,
	vector<pair<Interestpoint, Interestpoint>>	&ipMatching,
	SixPrmt	&spTmp
	)
{
	vector<Interestpoint> ipRef;
	Surf( strRefPath, ipRef );
	SetHeight( strRefPath, ipRef );

	vector<Interestpoint> ipIn;
	Surf( strInPath, ipIn );

	vector<pair<Interestpoint, Interestpoint>> ipPairTmp;

	getMatches( ipRef, ipIn, ipPairTmp );

	SetVector( ipPairTmp );
	eraseFarOutlier( ipPairTmp );

	CDib dibRef, dibIn;
	dibRef.Load( strRefPath );	dibIn.Load( strInPath );
	eraseMissMatchesUsingMean( ipPairTmp, dibRef, dibIn );

	if( ipPairTmp.size()>=3 )
		GetSixParameter( ipPairTmp, spTmp );
	else if( ipPairTmp.size()==0 )
	{
		printf( "관심점의 개수가 충분하지 않습니다.\n" );
		return;
	}
	for( unsigned int i=0; i<ipPairTmp.size(); i++ )
			ipMatching.push_back( ipPairTmp[i] );
}