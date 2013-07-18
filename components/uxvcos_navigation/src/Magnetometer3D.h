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

#ifndef __MAGNETOMETER3D_H__
#define __MAGNETOMETER3D_H__

#include <math.h>
#include <boost/numeric/ublas/matrix.hpp>

#include "WorldMagneticModel_2010.h"

namespace uxvcos {
namespace Navigation
{
	//! Class to handle magnetometer readings
	//!
	//! * Take 3D magnometric measurements in uT, recent rol and pitch angle
	//! * Take local geodetic position
	//! * calculate local declination
	//! * calculate magnetic and geodetic heading
	class TMAGNETOMETER3D
	{
		public:
			TMAGNETOMETER3D(void);
			virtual ~TMAGNETOMETER3D(void);

			//! Input data to this class
			//!
			//! \param *pBx, *pBy, *pBz pointer to 3D magnetic reading [uT]
			//! \param *pRol, *pPit     pointer to recent attitude angles [rad]
			void SetMeasurements(double* pBx, double* pBy, double* pBz, double* pRol, double* pPit);

			//! Get magnetic heading
			//!
			//! \return magnetic heading
			double GetMagHeading(void);

			//! Get geodetic heading
			//!
			//! \return geodetic heading
			double GetGeoHeading(void);

			//! Set local position
			//!
			//! \param pLat, pLon, pAlt pointer to local position
			void SetLocalPosition(double *pLat, double *pLon, double  *pAlt);

			//! Set declination
			//!
			//! \param pDeclination declination [rad]
			void SetDeclination(double declination);

			//! Get declination
			//!
			//! \return declination [rad]
			double GetDeclination(void);

			//! Get declination by position
			//!
			//! \param lat, lon, height local position
			//! \param year this year
			double GetDeclination(double lat, double lon, double height, double year);

			//! Get the normalized magnetic field vector for the current location
			//!
			//! \param pBxModel, pByModel, pBzModel
			void GetNormalizedMagnetFieldVector(double* pBxModel, double* pByModel, double* pBzModel);
			
			//! Get the normalized magnetic field vector for the current location
			//!
			//! \param lat, lon, height local position
			//! \param year this year
			//! \param pBxModel, pByModel, pBzModel
			void GetNormalizedMagnetFieldVector(double lat, double lon, double height, double year, double* pBxModel, double* pByModel, double* pBzModel);

		private:
			TWMM    WMM;
			double  MagHeading;
			double  Declination;
	};
}
} // namespace uxvcos

#endif
