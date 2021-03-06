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

#include "Magnetometer3D.h"


namespace uxvcos {
namespace Navigation
{
	using namespace boost::numeric::ublas;

	TMAGNETOMETER3D::TMAGNETOMETER3D()
		: MagHeading(0.0)
		, Declination(0.0)
	{
	}

	TMAGNETOMETER3D::~TMAGNETOMETER3D()
	{
	}

	void TMAGNETOMETER3D::SetMeasurements(double* pBx, double* pBy, double* pBz, double* pRol, double* pPit)
	{
		double Xh,Yh;

		// calculate compensated heading
		Xh = *pBx * cos(*pPit) + *pBy * sin(*pRol) * sin(*pPit) + *pBz * cos(*pRol) * sin(*pPit);
		Yh = *pBy * cos(*pRol) - *pBz * sin(*pRol);

		MagHeading = -atan2(Yh,Xh);
	}

	double TMAGNETOMETER3D::GetMagHeading(void)
	{
		return MagHeading;
	}

	double TMAGNETOMETER3D::GetGeoHeading(void)
	{
		return (MagHeading + Declination);
	}

	double TMAGNETOMETER3D::GetDeclination(void)
	{
		return Declination;
	}

	double TMAGNETOMETER3D::GetDeclination(double lat, double lon, double height, double year)
	{
		WMM.SetLocation(lat,lon,height);
		WMM.SetYear(year);
		WMM.Update();
		Declination = WMM.GetDeclination();

		return GetDeclination();
	}

	void TMAGNETOMETER3D::SetDeclination(double declination)
	{
		Declination = declination;
	}

	void TMAGNETOMETER3D::GetNormalizedMagnetFieldVector(double* pBxModel, double* pByModel, double* pBzModel)
	{
		//--> Get magnetic field vector from world magnetic model
		boost::numeric::ublas::vector<double> normalizedMagneticFieldVector = WMM.GetMagnetFieldVector();

		//--> Normalize magnetic field vector
		//-------------------------------------------------------------------------------------
		double norm = sqrt(normalizedMagneticFieldVector(0)*normalizedMagneticFieldVector(0) +
							normalizedMagneticFieldVector(1)*normalizedMagneticFieldVector(1) +
							normalizedMagneticFieldVector(2)*normalizedMagneticFieldVector(2));

		*pBxModel = normalizedMagneticFieldVector(0) / norm;
		*pByModel = normalizedMagneticFieldVector(1) / norm;
		*pBzModel = normalizedMagneticFieldVector(2) / norm;
		//-------------------------------------------------------------------------------------
	}

	void TMAGNETOMETER3D::GetNormalizedMagnetFieldVector(double lat, double lon, double height, double year, double* pBxModel, double* pByModel, double* pBzModel)
	{
		WMM.SetLocation(lat,lon,height);
		WMM.SetYear(year);
		WMM.Update();
		GetNormalizedMagnetFieldVector(pBxModel, pByModel, pBzModel);
	}
}

} // namespace uxvcos
