#ifndef __WMM_H__
#define __WMM_H__

#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>

namespace uxvcos {
namespace Navigation
{
	#define ELEMENT_LAT 0 // Vector Row Indices LAT/LON/ALT
	#define ELEMENT_LON 1 // Vector Row Indices LAT/LON/ALT
	#define ELEMENT_ALT 2 // Vector Row Indices LAT/LON/ALT
	#define ELEMENT_X   0 // Vector Row Indices X
	#define ELEMENT_Y   1 // Vector Row Indices Y
	#define ELEMENT_Z   2 // Vector Row Indices Z
	#define ELEMENT_N   0 // Vector Row Indices north
	#define ELEMENT_E   1 // Vector Row Indices east
	#define ELEMENT_D   2 // Vector Row Indices down

	// ---------------------------------------------------------
	// (world magnetic model 2010)
	// ---------------------------------------------------------
	#define WMM_SEMI_MAJOR_RADIUS   6378137.0000 // Earth semi-major axis
	#define WMM_SEMI_MINOR_RADIUS   6356752.3142 // Earth semi-minor axis
	#define WMM_SPHERICAL_RADIUS    6371200.0000 // Spherical radius

	// ---------------------------------------------------------
	// Class Twwm (world magnetic model 2010)
	// ---------------------------------------------------------
	//! Twmm class
	class TWMM
	{
		protected:
			boost::numeric::ublas::matrix<double> mGNM;				// [14, 14] Time-invariant Gauss coefficients
			boost::numeric::ublas::matrix<double> mHNM;				// [14, 14] Time-invariant Gauss coefficients
			boost::numeric::ublas::matrix<double> mGTNM;			// [14, 14] Time-variant Gauss coefficients
			boost::numeric::ublas::matrix<double> mHTNM;			// [14, 14] Time-variant Gauss coefficients
			boost::numeric::ublas::matrix<double> mRoots0;			// [14, 14] Gauss coefficient roots
			boost::numeric::ublas::matrix<double> mRoots1;			// [14, 14] Gauss coefficient inverse roots
			boost::numeric::ublas::vector<double> vRoots;			//     [14] Gauss coefficient index roots
			boost::numeric::ublas::vector<double> vLocation;		//     [ 3] Current geodetic location
			boost::numeric::ublas::vector<double> vMagneticField;	//     [ 3] Current magnetic field vector

			double sYear;											// Current fractional year

		private:
			void InitGeoMagModel(void);

		public:
			TWMM();
			~TWMM();

			void SetYear(double year);
			void SetLocation(double pLat, double pLon, double pAlt);

			void Update();

			double GetYear(void);
			double GetDeclination(void);
			boost::numeric::ublas::vector<double> GetMagnetFieldVector();
	};
}
} // namespace uxvcos

#endif
