//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
