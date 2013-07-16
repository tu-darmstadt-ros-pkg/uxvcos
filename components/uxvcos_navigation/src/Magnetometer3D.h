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
