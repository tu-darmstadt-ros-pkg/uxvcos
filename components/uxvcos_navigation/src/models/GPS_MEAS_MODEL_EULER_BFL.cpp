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

#include "GPS_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	GPS_MEAS_MODEL_EULER_BFL::GPS_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_GPS,NUM_COND_ARGUMENTS_MEASUREMENT_GPS)
	{
		
	}

	GPS_MEAS_MODEL_EULER_BFL::~GPS_MEAS_MODEL_EULER_BFL()
	{
		
	}
	
	//--> Measurement equation y = h(x)
	ColumnVector GPS_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);

		//--> Enhance visibility
		//----------------------------------------------------------	
		double Lat		= x(PX);
		double Lon		= x(PY);
		double v_n		= x(VX);
		double v_e		= x(VY);
		//----------------------------------------------------------

		ColumnVector y(NUMBER_OF_MEASUREMENTS_GPS);

		y(1) = Lat;
		y(2) = Lon;
		y(3) = v_n;
		y(4) = v_e;

		return y + AdditiveNoiseMuGet();
	}
	
	//--> Jacobian matrix H
	Matrix GPS_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
	{
		//--> Derivative to the first conditional argument (x)
		if (i==0)
		{
			//ColumnVector x = ConditionalArgumentGet(0);

			//--> Enhance visibility
			//----------------------------------------------------------

			//----------------------------------------------------------

			//--> Set H-Matrix
			//----------------------------------------------------------
			Matrix H(NUMBER_OF_MEASUREMENTS_GPS,NUMBER_OF_STATES);

			//--> Clear H-Matrix
			H = 0.0;

			H(1,PX) = 1.0;
			H(2,PY) = 1.0;
			H(3,VX) = 1.0;
			H(4,VY) = 1.0;
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
