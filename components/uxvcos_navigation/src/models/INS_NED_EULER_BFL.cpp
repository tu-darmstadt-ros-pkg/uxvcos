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

#include "INS_NED_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	INS_NED_EULER_BFL::INS_NED_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_STATES,NUM_COND_ARGUMENTS_SYSTEM)
	{
		dt = 1.0;
	}

	INS_NED_EULER_BFL::~INS_NED_EULER_BFL()
	{
		
	}
	
	//--> Set-Methods
	//----------------------------------------------------------
	void INS_NED_EULER_BFL::setDt(double dt)
	{
		this->dt = dt;
	}
	//----------------------------------------------------------
	
	//--> Get-Methods
	//----------------------------------------------------------
	double INS_NED_EULER_BFL::getDt()
	{
		return this->dt;
	}
	//----------------------------------------------------------

    //--> Returns covariance from system uncertainty and multiplies this with dt
	SymmetricMatrix INS_NED_EULER_BFL::CovarianceGet() const
	{
        // return ((this->AdditiveNoiseSigmaGet())*(dt*dt));
        return this->AdditiveNoiseSigmaGet()*dt;
    }

	//--> System equation of this model xpred = x(k+1) = f(x,u)
	ColumnVector INS_NED_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);
		ColumnVector u = ConditionalArgumentGet(1);
        
		//--> Enhance readability
		//----------------------------------------------------------
		double abx		= u(AX) + x(BIAS_AX);
		double aby		= u(AY) + x(BIAS_AY);
		double abz		= u(AZ) + x(BIAS_AZ);
		double wbx		= u(WX) + x(BIAS_WX);
		double wby		= u(WY) + x(BIAS_WY);
		double wbz		= u(WZ) + x(BIAS_WZ);
		
		double rol		= x(ROLL);
		double pit		= x(PITCH);
		double azi		= x(YAW);
		double p_x		= x(PX);
		double p_y		= x(PY);
		double p_z		= x(PZ);
		double v_x		= x(VX);
		double v_y		= x(VY);
		double v_z		= x(VZ);
		//----------------------------------------------------------

		//--> Speed up calculations
		//----------------------------------------------------------
		double sin_rol = sin(rol);
		double sin_pit = sin(pit);
		double sin_azi = sin(azi);
		double cos_rol = cos(rol);
		double cos_pit = cos(pit);
		double cos_azi = cos(azi);
		double tan_pit = tan(pit);
		//----------------------------------------------------------

		ColumnVector xpred(x);
		
		//--> Attitude (without return rotation rate)
		//----------------------------------------------------------
		xpred(ROLL)  = rol+dt*(wbx + (sin_rol*tan_pit)*wby + (cos_rol*tan_pit)*wbz);
		xpred(PITCH) = pit+dt*(              (cos_rol)*wby -         (sin_rol)*wbz);
		xpred(YAW)   = azi+dt*(      (sin_rol/cos_pit)*wby + (cos_rol/cos_pit)*wbz);
		//----------------------------------------------------------

		//--> Use new attitude angles and speed up calculations
		//----------------------------------------------------------
		// sin_rol = sin(xpred(ROLL));
		// sin_pit = sin(xpred(PITCH));
		// sin_azi = sin(xpred(YAW));
		// cos_rol = cos(xpred(ROLL));
		// cos_pit = cos(xpred(PITCH));
		// cos_azi = cos(xpred(YAW));
		// tan_pit = tan(xpred(PITCH));
		//----------------------------------------------------------

		//--> Velocity (without coriolis) and Position
		//----------------------------------------------------------
		if (!enableZVEL_NE_) {
			xpred(VX)  = v_x + dt*((cos_pit*cos_azi)*abx + (sin_rol*sin_pit*cos_azi-cos_rol*sin_azi)*aby + (cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*abz);
			xpred(VY)  = v_y + dt*((cos_pit*sin_azi)*abx + (sin_rol*sin_pit*sin_azi+cos_rol*cos_azi)*aby + (cos_rol*sin_pit*sin_azi-sin_rol*cos_azi)*abz);
			xpred(PX)  = p_x + dt*(v_x);
			xpred(PY)  = p_y + dt*(v_y);
		}

		if (!enableZVEL_D_) {
			xpred(VZ)  = v_z + dt*(       (-sin_pit)*abx +                         (sin_rol*cos_pit)*aby +                         (cos_rol*cos_pit)*abz + LocalGravity);
			xpred(PZ)  = p_z + dt*(v_z);
		}
		//----------------------------------------------------------

		//--> Limit angles of state vector
		//----------------------------------------------------------
		limit_anglerad(&xpred(ROLL)); // rol
		limit_anglerad(&xpred(PITCH), M_PI/2); // pit
		limit_anglerad(&xpred(YAW)); // azi
		//----------------------------------------------------------

		return xpred + AdditiveNoiseMuGet();
	}

	//--> Jacobian matrix A
	Matrix INS_NED_EULER_BFL::dfGet(unsigned int i) const
	{
		//--> Derivative to the first conditional argument (x)
		if (i==0)
		{
			ColumnVector x = ConditionalArgumentGet(0);
			ColumnVector u = ConditionalArgumentGet(1);

			//--> Enhance readability
			//----------------------------------------------------------
			double abx		= u(AX) + x(BIAS_AX);
			double aby		= u(AY) + x(BIAS_AY);
			double abz		= u(AZ) + x(BIAS_AZ);
			double wbx		= u(WX) + x(BIAS_WX);
			double wby		= u(WY) + x(BIAS_WY);
			double wbz		= u(WZ) + x(BIAS_WZ);

			double rol		= x(ROLL);
			double pit		= x(PITCH);
			double azi		= x(YAW);
			//----------------------------------------------------------
			
			//--> Speed up calculations
			//----------------------------------------------------------
			double sin_rol = sin(rol);
			double sin_pit = sin(pit);
			double sin_azi = sin(azi);
			double cos_rol = cos(rol);
			double cos_pit = cos(pit);
			double cos_azi = cos(azi);
			double tan_pit = tan(pit);
			double sin_rolDcos_pit = sin_rol/cos_pit;
			double cos_rolDcos_pit = cos_rol/cos_pit;

			double term1 = 1.0+tan_pit*tan_pit;
			double term2 = 1.0/(cos_pit*cos_pit);
			//----------------------------------------------------------

			//--> Set A-Matrix
			//----------------------------------------------------------
			Matrix A(NUMBER_OF_STATES,NUMBER_OF_STATES);

			//--> Clear A-Matrix and set eye
			A = 0.0;
			for (int i=1; i<=NUMBER_OF_STATES; i++) A(i,i) = 1.0;

			A(ROLL,ROLL)      = 1.0 + dt*(cos_rol*tan_pit*(wby)-sin_rol*tan_pit*(wbz));
			A(ROLL,PITCH)     = dt*(sin_rol*term1*(wby)+cos_rol*term1*(wbz));
			A(ROLL,BIAS_WX)   = dt;
			A(ROLL,BIAS_WY)   = dt*sin_rol*tan_pit;
			A(ROLL,BIAS_WZ)   = dt*cos_rol*tan_pit;

			A(PITCH,ROLL)     = dt*(-sin_rol*(wby)-cos_rol*(wbz));
			A(PITCH,PITCH)    = 1.0;
			A(PITCH,BIAS_WY)  = dt*cos_rol;
			A(PITCH,BIAS_WZ)  = -dt*sin_rol;

			A(YAW,ROLL)       = dt*(cos_rolDcos_pit*(wby)-sin_rolDcos_pit*(wbz));
			A(YAW,PITCH)      = dt*(sin_rol*term2*(wby)*sin_pit+cos_rol*term2*(wbz)*sin_pit);
			A(YAW,BIAS_WY)    = dt*sin_rolDcos_pit;
			A(YAW,BIAS_WZ)    = dt*cos_rolDcos_pit;

      if (!enableZVEL_NE_) {
        A(VX,ROLL)    = dt*((cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*(aby)+(-sin_rol*sin_pit*cos_azi+cos_rol*sin_azi)*(abz));
        A(VX,PITCH)   = dt*(-sin_pit*cos_azi*(abx)+sin_rol*cos_pit*cos_azi*(aby)+cos_rol*cos_pit*cos_azi*(abz));
        A(VX,YAW)     = dt*(-cos_pit*sin_azi*(abx)+(-sin_rol*sin_pit*sin_azi-cos_rol*cos_azi)*(aby)+(-cos_rol*sin_pit*sin_azi+sin_rol*cos_azi)*(abz));
        A(VX,BIAS_AX) = dt*(cos_pit*cos_azi);
        A(VX,BIAS_AY) = dt*(sin_rol*sin_pit*cos_azi-cos_rol*sin_azi);
        A(VX,BIAS_AZ) = dt*(cos_rol*sin_pit*cos_azi+sin_rol*sin_azi);

        A(VY,ROLL)     = dt*((cos_rol*sin_pit*sin_azi-sin_rol*cos_azi)*(aby)+(-sin_rol*sin_pit*sin_azi-cos_rol*cos_azi)*(abz));
        A(VY,PITCH)    = dt*(-sin_pit*sin_azi*(abx)+sin_rol*cos_pit*sin_azi*(aby)+cos_rol*cos_pit*sin_azi*(abz));
        A(VY,YAW)      = dt*(cos_pit*cos_azi*(abx)+(sin_rol*sin_pit*cos_azi-cos_rol*sin_azi)*(aby)+(cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*(abz));
        A(VY,BIAS_AX)  = dt*(cos_pit*sin_azi);
        A(VY,BIAS_AY)  = dt*(sin_rol*sin_pit*sin_azi+cos_rol*cos_azi);
        A(VY,BIAS_AZ)  = dt*(cos_rol*sin_pit*sin_azi-sin_rol*cos_azi);

        A(PX,VX)   = dt;
  //      A(PX,ROLL)     = A(PX,VX) * A(VX,ROLL);
  //      A(PX,PITCH)    = A(PX,VX) * A(VX,PITCH);
  //      A(PX,YAW)      = A(PX,VX) * A(VX,YAW);
  //      A(PX,BIAS_AX)  = A(PX,VX) * A(VX,BIAS_AX);
  //      A(PX,BIAS_AY)  = A(PX,VX) * A(VX,BIAS_AY);
  //      A(PX,BIAS_AZ)  = A(PX,VX) * A(VX,BIAS_AZ);

        A(PY,VY)   = dt;
  //      A(PY,ROLL)    = A(PY,VY) * A(VY,ROLL);
  //      A(PY,PITCH)   = A(PY,VY) * A(VY,PITCH);
  //      A(PY,YAW)     = A(PY,VY) * A(VY,YAW);
  //      A(PY,BIAS_AX) = A(PY,VY) * A(VY,BIAS_AX);
  //      A(PY,BIAS_AY) = A(PY,VY) * A(VY,BIAS_AY);
  //      A(PY,BIAS_AZ) = A(PY,VY) * A(VY,BIAS_AZ);
      }

      if (!enableZVEL_D_) {
        A(VZ,ROLL)     = dt*(cos_rol*cos_pit*(aby)-sin_rol*cos_pit*(abz));
        A(VZ,PITCH)    = dt*(-cos_pit*(abx)-sin_rol*sin_pit*(aby)-cos_rol*sin_pit*(abz));
        A(VZ,BIAS_AX)  = dt*(-sin_pit);
        A(VZ,BIAS_AY)  = dt*(sin_rol*cos_pit);
        A(VZ,BIAS_AZ)  = dt*(cos_rol*cos_pit);

        A(PZ,VZ)    = dt;
  //      A(PZ,ROLL)     = A(PZ,VZ) * A(VZ,ROLL);
  //      A(PZ,PITCH)    = A(PZ,VZ) * A(VZ,PITCH);
  //      A(PZ,BIAS_AX)  = A(PZ,VZ) * A(VZ,BIAS_AX);
  //      A(PZ,BIAS_AY)  = A(PZ,VZ) * A(VZ,BIAS_AY);
  //      A(PZ,BIAS_AZ)  = A(PZ,VZ) * A(VZ,BIAS_AZ);
      }
      //----------------------------------------------------------

			return A;
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
	
		void INS_NED_EULER_BFL::limit_anglerad(double* pAngle, double pMaxAngle) const
    {
      *pAngle -= 2.0*M_PI*floor(*pAngle/(2.0*M_PI) + 0.5);
      if (pMaxAngle != 0.0) {
        if (*pAngle >  pMaxAngle) *pAngle =  pMaxAngle;
        if (*pAngle < -pMaxAngle) *pAngle = -pMaxAngle;
      }
    }

		void INS_NED_EULER_BFL::setEarthData(const double pRmH, const double pRnH, const double pLocalGravity) {
			this->RmH = pRmH;
			this->RnH = pRnH;
			this->LocalGravity = pLocalGravity;
		}
}
} // namespace uxvocs
