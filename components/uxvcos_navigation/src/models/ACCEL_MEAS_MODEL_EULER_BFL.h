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

/*
 * ACCEL_MEAS_MODEL_EULER_BFL.h
 */

#ifndef _ACCEL_MEAS_MODEL_EULER_BFL_
#define _ACCEL_MEAS_MODEL_EULER_BFL_

/**
 * @brief First implementation of zero velocity measurement model with Orocos-BFL
 *
 * @author Martin Nowara
 *
 * @version 1.0
 *
 * @date 11/11/2010
 *
 */

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

#include "INS_NED_EULER_BFL.h"

namespace uxvcos {
namespace Navigation
{
	//--> Number of arguments arg0 = MeasurementVector [y]
	#define NUM_COND_ARGUMENTS_MEASUREMENT_ACCEL 1

	#ifndef NUMBER_OF_MEASUREMENTS_ACCEL
		#define NUMBER_OF_MEASUREMENTS_ACCEL 3
	#endif

	class ACCEL_MEAS_MODEL_EULER_BFL : public BFL::AnalyticConditionalGaussianAdditiveNoise
	{
		public:
			ACCEL_MEAS_MODEL_EULER_BFL();
			virtual ~ACCEL_MEAS_MODEL_EULER_BFL();

			//--> Measurement equation y = h(x)
			virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;

			//--> Jacobian matrix H
			virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

		private:
			static const double LocalGravity;
	};
}
} // namespace uxvocs

#endif
