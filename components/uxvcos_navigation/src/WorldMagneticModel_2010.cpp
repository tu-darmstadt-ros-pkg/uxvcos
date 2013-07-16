// ---------------------------------------------------------
// WMM Unit
// ---------------------------------------------------------
// UNIT DESCRIPTION:
// This unit contains the WMM class.  This class implements
// the World Magnetic Model (2010 version) to calculate the
// magnetic track from the true track the GPS receiver
// provides.
// ---------------------------------------------------------
// USAGE:
// The object is created using the constructor Create(). The
// object needs to be initialized using the Location and
// Year properties.  The magnetic declination can then be
// read using the Deviation property.
// ---------------------------------------------------------
// REVISIONS:
// 1.0  : Oliver Mittenzwei                         05/11/01
// 1.1  : Martin Nowara                             20/06/10
// ---------------------------------------------------------
#include "WorldMagneticModel_2010.h"

namespace uxvcos {
namespace Navigation
{
	using namespace boost::numeric::ublas;

	// ---------------------------------------------------------
	// CONSTRUCTOR TWMM.Create
	// ---------------------------------------------------------
	// DESCRIPTION:
	// The constructor creates the WMM object and initializes
	// all internal structures and variables.
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// None
	// ---------------------------------------------------------
	TWMM::TWMM()
	:mGNM(14,14),
	mHNM(14,14),
	mGTNM(14,14),
	mHTNM(14,14),
	mRoots0(14,14),
	mRoots1(14,14),
	vRoots(14),
	vLocation(3),
	vMagneticField(3)
	{
		vLocation(ELEMENT_LAT) = 0.0;
		vLocation(ELEMENT_LON) = 0.0;
		vLocation(ELEMENT_ALT) = 0.0;
		InitGeoMagModel();
	}

	// ---------------------------------------------------------
	// DESTRUCTOR TWMM.Destroy
	// ---------------------------------------------------------
	// DESCRIPTION:
	// Frees the WMM object.
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// None
	// ---------------------------------------------------------
	TWMM::~TWMM()
	{
	}

	// ---------------------------------------------------------
	// FUNCTION Update
	// ---------------------------------------------------------
	// DESCRIPTION:
	// This function calculates the magnetic field vector for
	// the current location
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// None
	// ---------------------------------------------------------
	void TWMM::Update()
	{
		matrix<double> P(14,14);    // Schmidt semi-normalized Associated Legendre coefficients
		matrix<double> DP(14,14);   // Derivatives of Legendre coefficients
		matrix<double> GNM(14,14);  // Current Gauss coefficients
		matrix<double> HNM(14,14);  // Current Gauss coefficients
		unsigned int n;				// Counter
		unsigned int m;				// Counter
		double slat;				// Latitude sine component
		double clat;				// Latitude cosine component
		vector<double> slon(14);	// Longitude sine component
		vector<double> clon(14);	// Longitude cosine component
		double theta;				// Theta
		double stheta;				// Theta sine component
		double ctheta;				// Theta cosine component
		double psi;					// Psi
		double spsi;				// Psi sine component
		double cpsi;				// Psi cosine component
		double inv;					// Helper
		double Radius;				// Radius at current position
		double LocalRadius;			// Local earth radius at tangent plane
		double Delta;				// Time difference since model start (05/01/01) (s)
		double MagRadius;			// Magnetic spherical radius
		double MagTheta;			// Magnetic theta
		double MagPhi;				// Magnetic phi
		double RadiusFactor0;		// Helper
		double RadiusFactor;		// Helper
		double Temp;				// Helper
		double AccuR;				// Accumulator for magnetic field calculations
		double AccuTheta;			// Accumulator for magnetic field calculations
		double AccuPhi;				// Accumulator for magnetic field calculations

		slat = sin(vLocation(ELEMENT_LAT));
		clat = cos(vLocation(ELEMENT_LAT));
		LocalRadius = sqrt(pow(WMM_SEMI_MAJOR_RADIUS * clat,2) + pow(WMM_SEMI_MINOR_RADIUS * slat,2));
		theta = atan2(clat * (vLocation(ELEMENT_ALT) * LocalRadius + pow(WMM_SEMI_MAJOR_RADIUS,2)),
					  slat * (vLocation(ELEMENT_ALT) * LocalRadius + pow(WMM_SEMI_MINOR_RADIUS,2)));
		ctheta = cos(theta);
		stheta = sin(theta);
		Radius = sqrt(pow(vLocation(ELEMENT_ALT),2) + 2 * vLocation(ELEMENT_ALT) * LocalRadius +
					 (pow(WMM_SEMI_MAJOR_RADIUS,4) - (pow(WMM_SEMI_MAJOR_RADIUS,4) -
					  pow(WMM_SEMI_MINOR_RADIUS,4)) * pow(slat,2)) /
					 (pow(WMM_SEMI_MAJOR_RADIUS,2) - (pow(WMM_SEMI_MAJOR_RADIUS,2) -
					  pow(WMM_SEMI_MINOR_RADIUS,2)) * pow(slat,2)));

		if (fabs(stheta) != 0)
			inv = 1.0/stheta;
		else
		{
			if (stheta > 0)
				inv =  1e8;
			else
				inv = -1e8;
		}

		P.clear();
		DP.clear();
		P(0,0) = 1.0;
		P(1,1) = stheta;
		DP(0,0)= 0.0;
		DP(1,1)= ctheta;
		P(1,0) = ctheta;
		DP(1,0)= -stheta;

		for (n=2; n<14; n++)
		{
			P(n,n) = P(n-1, n-1) * stheta * vRoots(n);
			DP(n,n)= (DP(n-1,n-1)* stheta + P(n-1,n-1) * ctheta) * vRoots(n);
		}

		for(m=0; m<14; m++)
		{
			int init;

			if (2 < m+1)
				init = m+1;
			else
				init = 2;

			for(n=init; n<14; n++)
			{
				P(n,m) = (P(n-1,m) * ctheta * (2 * n - 1) - P(n-2,m) * mRoots0(m,n)) * mRoots1(m,n);
				DP(n,m)= ((DP(n-1,m) * ctheta - P(n-1,m) * stheta) * (2 * n - 1) - DP(n-2,m) * mRoots0(m,n)) * mRoots1(m,n);
			}
		}

		Delta = (sYear-2010); //--> for WMM 2010; adjust this and mGNM/mHNM constants when WMM 2015 becomes available

		for(m=0; m<14; m++)
		{
			for(n=0; n<14; n++)
			{
				GNM(m,n) = mGNM(m,n) + Delta * mGTNM(m,n);
				HNM(m,n) = mHNM(m,n) + Delta * mHTNM(m,n);
			}
		}

		for(m=0; m<14; m++)
		{
			slon(m) = sin(m * vLocation(ELEMENT_LON));
			clon(m) = cos(m * vLocation(ELEMENT_LON));
		}

		MagRadius = 0;
		MagTheta  = 0;
		MagPhi    = 0;
		RadiusFactor0 = WMM_SPHERICAL_RADIUS / Radius;
		RadiusFactor = pow(RadiusFactor0,2);

		for(m=1; m<14; m++)
		{
			AccuR     = 0;
			AccuTheta = 0;
			AccuPhi   = 0;

			for(n=0; n<=m; n++)
			{
				Temp       = GNM(m,n) * clon(n) + HNM(m,n) * slon(n);
				AccuR     += Temp * P(m,n);
				AccuTheta += Temp * DP(m,n);
				AccuPhi   += n * (GNM(m,n) * slon(n) - HNM(m,n) * clon(n)) * P(m,n);
			}

			RadiusFactor = RadiusFactor * RadiusFactor0;
			MagRadius   += (m+1) * AccuR * RadiusFactor;
			MagTheta    += -AccuTheta * RadiusFactor;
			MagPhi      += AccuPhi * RadiusFactor * inv;
		}

		psi = theta - (0.5 * M_PI - vLocation(ELEMENT_LAT));
		spsi = sin(psi);
		cpsi = cos(psi);
		vMagneticField(ELEMENT_X) = -MagTheta * cpsi - MagRadius * spsi;
		vMagneticField(ELEMENT_Y) =  MagPhi;
		vMagneticField(ELEMENT_Z) =  MagTheta * spsi - MagRadius * cpsi;
	}

	// ---------------------------------------------------------
	// FUNCTION TWMM.GetDeclination
	// ---------------------------------------------------------
	// DESCRIPTION:
	// This functions returns the declination (offset from true
	// north) at the current position and time.
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// Declination (rad)
	// ---------------------------------------------------------
	double TWMM::GetDeclination(void)
	{
		if ((vMagneticField(ELEMENT_X) == 0) && (vMagneticField(ELEMENT_Y) == 0))
			return 0.0;
		else
			return atan2(vMagneticField(ELEMENT_Y),vMagneticField(ELEMENT_X));
	}

	// ---------------------------------------------------------
	// FUNCTION GetMagnetFieldVector
	// ---------------------------------------------------------
	// DESCRIPTION:
	// This function returns the magnetic field vector for
	// the current location
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// Magnetic field vector for current location
	// ---------------------------------------------------------
	vector<double> TWMM::GetMagnetFieldVector()
	{
		return vMagneticField;
	}

	// ---------------------------------------------------------
	// FUNCTION TWMM.InitGeoMagModel
	// ---------------------------------------------------------
	// DESCRIPTION:
	// This initializes the Gauss constants of the WMM 2010 and
	// performs some precalculations needed later in the WMM
	// calculations.
	// ---------------------------------------------------------
	// PARAMETERS:
	// None
	// ---------------------------------------------------------
	// RETURN VALUE:
	// None
	// ---------------------------------------------------------
	void TWMM::InitGeoMagModel(void)
	{
		unsigned int m,n; // Counter

		mGNM.clear();
		mHNM.clear();
		mGTNM.clear();
		mHTNM.clear();

		mGNM( 1 , 0) =  -29496.6; mHNM( 1 , 0) =      0.0; mGTNM( 1 , 0) =      11.6; mHTNM( 1 , 0) =       0.0;
		mGNM( 1 , 1) =   -1586.3; mHNM( 1 , 1) =   4944.4; mGTNM( 1 , 1) =      16.5; mHTNM( 1 , 1) =     -25.9;
		mGNM( 2 , 0) =   -2396.6; mHNM( 2 , 0) =      0.0; mGTNM( 2 , 0) =     -12.1; mHTNM( 2 , 0) =       0.0;
		mGNM( 2 , 1) =    3026.1; mHNM( 2 , 1) =  -2707.7; mGTNM( 2 , 1) =      -4.4; mHTNM( 2 , 1) =     -22.5;
		mGNM( 2 , 2) =    1668.6; mHNM( 2 , 2) =   -576.1; mGTNM( 2 , 2) =       1.9; mHTNM( 2 , 2) =     -11.8;
		mGNM( 3 , 0) =    1340.1; mHNM( 3 , 0) =      0.0; mGTNM( 3 , 0) =       0.4; mHTNM( 3 , 0) =       0.0;
		mGNM( 3 , 1) =   -2326.2; mHNM( 3 , 1) =   -160.2; mGTNM( 3 , 1) =      -4.1; mHTNM( 3 , 1) =       7.3;
		mGNM( 3 , 2) =    1231.9; mHNM( 3 , 2) =    251.9; mGTNM( 3 , 2) =      -2.9; mHTNM( 3 , 2) =      -3.9;
		mGNM( 3 , 3) =     634.0; mHNM( 3 , 3) =   -536.6; mGTNM( 3 , 3) =      -7.7; mHTNM( 3 , 3) =      -2.6;
		mGNM( 4 , 0) =     912.6; mHNM( 4 , 0) =      0.0; mGTNM( 4 , 0) =      -1.8; mHTNM( 4 , 0) =       0.0;
		mGNM( 4 , 1) =     808.9; mHNM( 4 , 1) =    286.4; mGTNM( 4 , 1) =       2.3; mHTNM( 4 , 1) =       1.1;
		mGNM( 4 , 2) =     166.7; mHNM( 4 , 2) =   -211.2; mGTNM( 4 , 2) =      -8.7; mHTNM( 4 , 2) =       2.7;
		mGNM( 4 , 3) =    -357.1; mHNM( 4 , 3) =    164.3; mGTNM( 4 , 3) =       4.6; mHTNM( 4 , 3) =       3.9;
		mGNM( 4 , 4) =      89.4; mHNM( 4 , 4) =   -309.1; mGTNM( 4 , 4) =      -2.1; mHTNM( 4 , 4) =      -0.8;
		mGNM( 5 , 0) =    -230.9; mHNM( 5 , 0) =      0.0; mGTNM( 5 , 0) =      -1.0; mHTNM( 5 , 0) =       0.0;
		mGNM( 5 , 1) =     357.2; mHNM( 5 , 1) =     44.6; mGTNM( 5 , 1) =       0.6; mHTNM( 5 , 1) =       0.4;
		mGNM( 5 , 2) =     200.3; mHNM( 5 , 2) =    188.9; mGTNM( 5 , 2) =      -1.8; mHTNM( 5 , 2) =       1.8;
		mGNM( 5 , 3) =    -141.1; mHNM( 5 , 3) =   -118.2; mGTNM( 5 , 3) =      -1.0; mHTNM( 5 , 3) =       1.2;
		mGNM( 5 , 4) =    -163.0; mHNM( 5 , 4) =      0.0; mGTNM( 5 , 4) =       0.9; mHTNM( 5 , 4) =       4.0;
		mGNM( 5 , 5) =      -7.8; mHNM( 5 , 5) =    100.9; mGTNM( 5 , 5) =       1.0; mHTNM( 5 , 5) =      -0.6;
		mGNM( 6 , 0) =      72.8; mHNM( 6 , 0) =      0.0; mGTNM( 6 , 0) =      -0.2; mHTNM( 6 , 0) =       0.0;
		mGNM( 6 , 1) =      68.6; mHNM( 6 , 1) =    -20.8; mGTNM( 6 , 1) =      -0.2; mHTNM( 6 , 1) =      -0.2;
		mGNM( 6 , 2) =      76.0; mHNM( 6 , 2) =     44.1; mGTNM( 6 , 2) =      -0.1; mHTNM( 6 , 2) =      -2.1;
		mGNM( 6 , 3) =    -141.4; mHNM( 6 , 3) =     61.5; mGTNM( 6 , 3) =       2.0; mHTNM( 6 , 3) =      -0.4;
		mGNM( 6 , 4) =     -22.8; mHNM( 6 , 4) =    -66.3; mGTNM( 6 , 4) =      -1.7; mHTNM( 6 , 4) =      -0.6;
		mGNM( 6 , 5) =      13.2; mHNM( 6 , 5) =      3.1; mGTNM( 6 , 5) =      -0.3; mHTNM( 6 , 5) =       0.5;
		mGNM( 6 , 6) =     -77.9; mHNM( 6 , 6) =     55.0; mGTNM( 6 , 6) =       1.7; mHTNM( 6 , 6) =       0.9;
		mGNM( 7 , 0) =      80.5; mHNM( 7 , 0) =      0.0; mGTNM( 7 , 0) =       0.1; mHTNM( 7 , 0) =       0.0;
		mGNM( 7 , 1) =     -75.1; mHNM( 7 , 1) =    -57.9; mGTNM( 7 , 1) =      -0.1; mHTNM( 7 , 1) =       0.7;
		mGNM( 7 , 2) =      -4.7; mHNM( 7 , 2) =    -21.1; mGTNM( 7 , 2) =      -0.6; mHTNM( 7 , 2) =       0.3;
		mGNM( 7 , 3) =      45.3; mHNM( 7 , 3) =      6.5; mGTNM( 7 , 3) =       1.3; mHTNM( 7 , 3) =      -0.1;
		mGNM( 7 , 4) =      13.9; mHNM( 7 , 4) =     24.9; mGTNM( 7 , 4) =       0.4; mHTNM( 7 , 4) =      -0.1;
		mGNM( 7 , 5) =      10.4; mHNM( 7 , 5) =      7.0; mGTNM( 7 , 5) =       0.3; mHTNM( 7 , 5) =      -0.8;
		mGNM( 7 , 6) =       1.7; mHNM( 7 , 6) =    -27.7; mGTNM( 7 , 6) =      -0.7; mHTNM( 7 , 6) =      -0.3;
		mGNM( 7 , 7) =       4.9; mHNM( 7 , 7) =     -3.3; mGTNM( 7 , 7) =       0.6; mHTNM( 7 , 7) =       0.3;
		mGNM( 8 , 0) =      24.4; mHNM( 8 , 0) =      0.0; mGTNM( 8 , 0) =      -0.1; mHTNM( 8 , 0) =       0.0;
		mGNM( 8 , 1) =       8.1; mHNM( 8 , 1) =     11.0; mGTNM( 8 , 1) =       0.1; mHTNM( 8 , 1) =      -0.1;
		mGNM( 8 , 2) =     -14.5; mHNM( 8 , 2) =    -20.0; mGTNM( 8 , 2) =      -0.6; mHTNM( 8 , 2) =       0.2;
		mGNM( 8 , 3) =      -5.6; mHNM( 8 , 3) =     11.9; mGTNM( 8 , 3) =       0.2; mHTNM( 8 , 3) =       0.4;
		mGNM( 8 , 4) =     -19.3; mHNM( 8 , 4) =    -17.4; mGTNM( 8 , 4) =      -0.2; mHTNM( 8 , 4) =       0.4;
		mGNM( 8 , 5) =      11.5; mHNM( 8 , 5) =     16.7; mGTNM( 8 , 5) =       0.3; mHTNM( 8 , 5) =       0.1;
		mGNM( 8 , 6) =      10.9; mHNM( 8 , 6) =      7.0; mGTNM( 8 , 6) =       0.3; mHTNM( 8 , 6) =      -0.1;
		mGNM( 8 , 7) =     -14.1; mHNM( 8 , 7) =    -10.8; mGTNM( 8 , 7) =      -0.6; mHTNM( 8 , 7) =       0.4;
		mGNM( 8 , 8) =      -3.7; mHNM( 8 , 8) =      1.7; mGTNM( 8 , 8) =       0.2; mHTNM( 8 , 8) =       0.3;
		mGNM( 9 , 0) =       5.4; mHNM( 9 , 0) =      0.0; mGTNM( 9 , 0) =       0.0; mHTNM( 9 , 0) =       0.0;
		mGNM( 9 , 1) =       9.4; mHNM( 9 , 1) =    -20.5; mGTNM( 9 , 1) =      -0.1; mHTNM( 9 , 1) =       0.0;
		mGNM( 9 , 2) =       3.4; mHNM( 9 , 2) =     11.5; mGTNM( 9 , 2) =       0.0; mHTNM( 9 , 2) =      -0.2;
		mGNM( 9 , 3) =      -5.2; mHNM( 9 , 3) =     12.8; mGTNM( 9 , 3) =       0.3; mHTNM( 9 , 3) =       0.0;
		mGNM( 9 , 4) =       3.1; mHNM( 9 , 4) =     -7.2; mGTNM( 9 , 4) =      -0.4; mHTNM( 9 , 4) =      -0.1;
		mGNM( 9 , 5) =     -12.4; mHNM( 9 , 5) =     -7.4; mGTNM( 9 , 5) =      -0.3; mHTNM( 9 , 5) =       0.1;
		mGNM( 9 , 6) =      -0.7; mHNM( 9 , 6) =      8.0; mGTNM( 9 , 6) =       0.1; mHTNM( 9 , 6) =       0.0;
		mGNM( 9 , 7) =       8.4; mHNM( 9 , 7) =      2.1; mGTNM( 9 , 7) =      -0.1; mHTNM( 9 , 7) =      -0.2;
		mGNM( 9 , 8) =      -8.5; mHNM( 9 , 8) =     -6.1; mGTNM( 9 , 8) =      -0.4; mHTNM( 9 , 8) =       0.3;
		mGNM( 9 , 9) =     -10.1; mHNM( 9 , 9) =      7.0; mGTNM( 9 , 9) =      -0.2; mHTNM( 9 , 9) =       0.2;
		mGNM(10 , 0) =      -2.0; mHNM(10 , 0) =      0.0; mGTNM(10 , 0) =       0.0; mHTNM(10 , 0) =       0.0;
		mGNM(10 , 1) =      -6.3; mHNM(10 , 1) =      2.8; mGTNM(10 , 1) =       0.0; mHTNM(10 , 1) =       0.1;
		mGNM(10 , 2) =       0.9; mHNM(10 , 2) =     -0.1; mGTNM(10 , 2) =      -0.1; mHTNM(10 , 2) =      -0.1;
		mGNM(10 , 3) =      -1.1; mHNM(10 , 3) =      4.7; mGTNM(10 , 3) =       0.2; mHTNM(10 , 3) =       0.0;
		mGNM(10 , 4) =      -0.2; mHNM(10 , 4) =      4.4; mGTNM(10 , 4) =       0.0; mHTNM(10 , 4) =      -0.1;
		mGNM(10 , 5) =       2.5; mHNM(10 , 5) =     -7.2; mGTNM(10 , 5) =      -0.1; mHTNM(10 , 5) =      -0.1;
		mGNM(10 , 6) =      -0.3; mHNM(10 , 6) =     -1.0; mGTNM(10 , 6) =      -0.2; mHTNM(10 , 6) =       0.0;
		mGNM(10 , 7) =       2.2; mHNM(10 , 7) =     -3.9; mGTNM(10 , 7) =       0.0; mHTNM(10 , 7) =      -0.1;
		mGNM(10 , 8) =       3.1; mHNM(10 , 8) =     -2.0; mGTNM(10 , 8) =      -0.1; mHTNM(10 , 8) =      -0.2;
		mGNM(10 , 9) =      -1.0; mHNM(10 , 9) =     -2.0; mGTNM(10 , 9) =      -0.2; mHTNM(10 , 9) =       0.0;
		mGNM(10 ,10) =      -2.8; mHNM(10 ,10) =     -8.3; mGTNM(10 ,10) =      -0.2; mHTNM(10 ,10) =      -0.1;
		mGNM(11 , 0) =       3.0; mHNM(11 , 0) =      0.0; mGTNM(11 , 0) =       0.0; mHTNM(11 , 0) =       0.0;
		mGNM(11 , 1) =      -1.5; mHNM(11 , 1) =      0.2; mGTNM(11 , 1) =       0.0; mHTNM(11 , 1) =       0.0;
		mGNM(11 , 2) =      -2.1; mHNM(11 , 2) =      1.7; mGTNM(11 , 2) =       0.0; mHTNM(11 , 2) =       0.1;
		mGNM(11 , 3) =       1.7; mHNM(11 , 3) =     -0.6; mGTNM(11 , 3) =       0.1; mHTNM(11 , 3) =       0.0;
		mGNM(11 , 4) =      -0.5; mHNM(11 , 4) =     -1.8; mGTNM(11 , 4) =       0.0; mHTNM(11 , 4) =       0.1;
		mGNM(11 , 5) =       0.5; mHNM(11 , 5) =      0.9; mGTNM(11 , 5) =       0.0; mHTNM(11 , 5) =       0.0;
		mGNM(11 , 6) =      -0.8; mHNM(11 , 6) =     -0.4; mGTNM(11 , 6) =       0.0; mHTNM(11 , 6) =       0.1;
		mGNM(11 , 7) =       0.4; mHNM(11 , 7) =     -2.5; mGTNM(11 , 7) =       0.0; mHTNM(11 , 7) =       0.0;
		mGNM(11 , 8) =       1.8; mHNM(11 , 8) =     -1.3; mGTNM(11 , 8) =       0.0; mHTNM(11 , 8) =      -0.1;
		mGNM(11 , 9) =       0.1; mHNM(11 , 9) =     -2.1; mGTNM(11 , 9) =       0.0; mHTNM(11 , 9) =      -0.1;
		mGNM(11 ,10) =       0.7; mHNM(11 ,10) =     -1.9; mGTNM(11 ,10) =      -0.1; mHTNM(11 ,10) =       0.0;
		mGNM(11 ,11) =       3.8; mHNM(11 ,11) =     -1.8; mGTNM(11 ,11) =       0.0; mHTNM(11 ,11) =      -0.1;
		mGNM(12 , 0) =      -2.2; mHNM(12 , 0) =      0.0; mGTNM(12 , 0) =       0.0; mHTNM(12 , 0) =       0.0;
		mGNM(12 , 1) =      -0.2; mHNM(12 , 1) =     -0.9; mGTNM(12 , 1) =       0.0; mHTNM(12 , 1) =       0.0;
		mGNM(12 , 2) =       0.3; mHNM(12 , 2) =      0.3; mGTNM(12 , 2) =       0.1; mHTNM(12 , 2) =       0.0;
		mGNM(12 , 3) =       1.0; mHNM(12 , 3) =      2.1; mGTNM(12 , 3) =       0.1; mHTNM(12 , 3) =       0.0;
		mGNM(12 , 4) =      -0.6; mHNM(12 , 4) =     -2.5; mGTNM(12 , 4) =      -0.1; mHTNM(12 , 4) =       0.0;
		mGNM(12 , 5) =       0.9; mHNM(12 , 5) =      0.5; mGTNM(12 , 5) =       0.0; mHTNM(12 , 5) =       0.0;
		mGNM(12 , 6) =      -0.1; mHNM(12 , 6) =      0.6; mGTNM(12 , 6) =       0.0; mHTNM(12 , 6) =       0.1;
		mGNM(12 , 7) =       0.5; mHNM(12 , 7) =      0.0; mGTNM(12 , 7) =       0.0; mHTNM(12 , 7) =       0.0;
		mGNM(12 , 8) =      -0.4; mHNM(12 , 8) =      0.1; mGTNM(12 , 8) =       0.0; mHTNM(12 , 8) =       0.0;
		mGNM(12 , 9) =      -0.4; mHNM(12 , 9) =      0.3; mGTNM(12 , 9) =       0.0; mHTNM(12 , 9) =       0.0;
		mGNM(12 ,10) =       0.2; mHNM(12 ,10) =     -0.9; mGTNM(12 ,10) =       0.0; mHTNM(12 ,10) =       0.0;
		mGNM(12 ,11) =      -0.8; mHNM(12 ,11) =     -0.2; mGTNM(12 ,11) =      -0.1; mHTNM(12 ,11) =       0.0;
		mGNM(12 ,12) =       0.0; mHNM(12 ,12) =      0.9; mGTNM(12 ,12) =       0.1; mHTNM(12 ,12) =       0.0;

		for(n=2; n<14; n++)
		{
			vRoots(n) = sqrt((2.0 * n - 1.0) / (2.0 * n));
		}

		for(m=0; m<14; m++)
		{
			int init;

			if (2 < m+1)
				init = m+1;
			else
				init = 2;

			for(n=init; n<14; n++)
			{
				mRoots0(m,n) = sqrt((double) (n-1)*(n-1) - (double) m*m);
				mRoots1(m,n) = 1.0 / sqrt((double) n*n - (double) m*m);
			}
		}
	}

	void TWMM::SetLocation(double pLat, double pLon, double pAlt)
	{
		vLocation(ELEMENT_LAT) = pLat;
		vLocation(ELEMENT_LON) = pLon;
		vLocation(ELEMENT_ALT) = pAlt;
	}

	double TWMM::GetYear(void)
	{
		return sYear;
	}

	void TWMM::SetYear(double year)
	{
		sYear = year;
	}
}
} // namespace uxvcos
