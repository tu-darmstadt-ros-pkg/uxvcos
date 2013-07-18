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
 * INS_NED_EULER_BFL.h
 */

#ifndef _INS_NED_EULER_BFL_
#define _INS_NED_EULER_BFL_

/**
 * @brief First implementation of a NED-Navigation filter with Orocos-BFL
 *
 * @author Martin Nowara
 *
 * @version 1.0
 *
 * @date 27/10/2010
 *
 */

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace uxvcos {
namespace Navigation
{
	//--> Number of arguments arg0 = StateVector [x], arg1 = InputVector [u]
	#define NUM_COND_ARGUMENTS_SYSTEM 2
	#define NUMBER_OF_INPUTS 6
	
	#ifndef NUMBER_OF_STATES
		#define NUMBER_OF_STATES 15
	#endif 

	enum StateIndex { ROLL = 1, PITCH, YAW,
                      PX, PY, PZ,
                      VX, VY, VZ,
                      BIAS_AX, BIAS_AY, BIAS_AZ,
                      BIAS_WX, BIAS_WY, BIAS_WZ };
	enum InputIndex { AX = 1, AY, AZ, WX, WY, WZ };

	class INS_NED_EULER_BFL : public BFL::AnalyticConditionalGaussianAdditiveNoise
	{
		public:
			INS_NED_EULER_BFL();
			virtual ~INS_NED_EULER_BFL();

			//--> Set-Methods
			void setDt(double dt);
			
			//--> Get-Methods
			double getDt();

			//--> Returns covariance from system uncertainty and multiplies this with dt^2
			virtual MatrixWrapper::SymmetricMatrix CovarianceGet() const;
			
			//--> System equation of this model xpred = x(k+1) = f(x,u)
			virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
			
			//--> Jacobian matrix A
			virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

			void setEarthData(const double RmH, const double RnH, const double LocalGravity);

			void enableZVEL_NE(bool truefalse) { enableZVEL_NE_ = truefalse; }
			void enableZVEL_D (bool truefalse) { enableZVEL_D_  = truefalse; }

		protected:
			double dt;

			void limit_anglerad(double* pAngle, double pMaxAngle = 0.0) const;

	private:
			double RmH;
			double RnH;
			double LocalGravity;

			bool	 enableZVEL_NE_;
			bool	 enableZVEL_D_;
	};
}
} // namespace uxvocs

#endif
