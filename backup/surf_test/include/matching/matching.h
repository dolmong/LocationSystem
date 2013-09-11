
#pragma once

#include <vector>
#include <math.h>

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
	void getMatches( std::vector<Interestpoint> &SavePrmt1, std::vector<Interestpoint> &SavePrmt2, std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec );
	void getMatchesInlineFunc( std::vector<Interestpoint> &Prmt1, std::vector<Interestpoint> &Prmt2, std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec, bool chkchg );
	void erasePt( std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec, std::vector<int> &PtIndx );
	int chkScale( float scale );
	float calculMean( int x, int y, int masksize, int width, int height, BYTE** ptr );
	void SetVector( std:::vector<pair<Interestpoint, Interestpoint>> &ipMatching );
	bool SetHeight( std::string strPath, std::vector<Interestpoint> &itpointTmp );
	void eraseFarOutlier( std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec);	// Matching 제거 함수
	void eraseMissMatchesUsingMean( std::vector<pair<Interestpoint, Interestpoint>> &IpPairVec, CDib& dibRef, CDib& dibIn );	// Matching 제거 함수
	void EraseMissDis( std::vector<pair<Interestpoint, Interestpoint>> &ipMatching, float Desired );	// Matching 제거 함수
	void GetSixParameter( std::vector<pair<Interestpoint, Interestpoint>> &IpPaitrVec, SixPrmt &spTmp );
	void DoLeastSquare_Affine( std::vector<pair<Interestpoint, Interestpoint>> &IpPairVecL, SixPrmt &spTmp );

private:
	// Variable
	std::vector<ErrValue> m_ErrValue;

public:
	// Function
	void Matching( CString strRefPath, CString strInPath, vector<pair<Interestpoint, Interestpoint>> &ipMatching, SixPrmt &spTmp );

public:
	// Variable
	std::vector<pair<Interestpoint, Interestpoint>> ipPairMat;
	SixPrmt spMeter;
};