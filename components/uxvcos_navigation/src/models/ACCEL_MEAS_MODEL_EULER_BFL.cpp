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

#include "ACCEL_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	const double ACCEL_MEAS_MODEL_EULER_BFL::LocalGravity = -9.8065;

	ACCEL_MEAS_MODEL_EULER_BFL::ACCEL_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_ACCEL,NUM_COND_ARGUMENTS_MEASUREMENT_ACCEL)
	{

	}

	ACCEL_MEAS_MODEL_EULER_BFL::~ACCEL_MEAS_MODEL_EULER_BFL()
	{

	}

	//--> Measurement equation y = h(x)
	ColumnVector ACCEL_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);
		ColumnVector y(NUMBER_OF_MEASUREMENTS_ACCEL);

		//----------------------------------------------------------
		double rol	   = x(ROLL);
		double pit	   = x(PITCH);
		double sin_rol = sin(rol);
		double sin_pit = sin(pit);
		double cos_rol = cos(rol);
		double cos_pit = cos(pit);
		//----------------------------------------------------------

		y(1) =  (        sin_pit)*LocalGravity - x(BIAS_AX);
		y(2) = -(sin_rol*cos_pit)*LocalGravity - x(BIAS_AY);
		y(3) = -(cos_rol*cos_pit)*LocalGravity - x(BIAS_AZ);

		return y + AdditiveNoiseMuGet();
	}

	//--> Jacobian matrix H
	Matrix ACCEL_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
	{
	  ColumnVector x = ConditionalArgumentGet(0);

	  //--> Derivative to the first conditional argument (x)
		if (i==0)
		{
			//ColumnVector x = ConditionalArgumentGet(0);

			//--> Enhance visibility
			//----------------------------------------------------------

			//----------------------------------------------------------

			//--> Set H-Matrix
			//----------------------------------------------------------
			Matrix H(NUMBER_OF_MEASUREMENTS_ACCEL,NUMBER_OF_STATES);

			//----------------------------------------------------------
			double rol	   = x(ROLL);
			double pit	   = x(PITCH);
			double sin_rol = sin(rol);
			double sin_pit = sin(pit);
			double cos_rol = cos(rol);
			double cos_pit = cos(pit);
			//----------------------------------------------------------

			//--> Clear H-Matrix
			H = 0.0;

			H(1,ROLL)    =  0.0;
			H(1,PITCH)   =  (        cos_pit)*LocalGravity;
			// H(1,BIAS_AX) = -1.0;

			H(2,ROLL)    = -(cos_rol*cos_pit)*LocalGravity;
			H(2,PITCH)   =  (sin_rol*sin_pit)*LocalGravity;
			// H(2,BIAS_AY) = -1.0;

			H(3,ROLL)    =  (sin_rol*cos_pit)*LocalGravity;
			H(3,PITCH)   =  (cos_rol*sin_pit)*LocalGravity;
			H(3,BIAS_AZ) = -1.0;

			//----------------------------------------------------------

			return H;
		}
		else
		{
			if (i >= NumConditionalArgumentsGet())
			{
				cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
				exit(-BFL_ERRMISUSE);
			}
			else
			{
				cerr << "The df is not implemented for the" << i << "th conditional argument\n";
				exit(-BFL_ERRMISUSE);
			}
	}
    }
}
} // namespace uxvocs
