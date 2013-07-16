#include "MAG_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	MAG_MEAS_MODEL_EULER_BFL::MAG_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_MAG,NUM_COND_ARGUMENTS_MEASUREMENT_MAG)
	, normalizedMagneticFieldVector(3)
	{
		normalizedMagneticFieldVector(1) = 1.0;
		normalizedMagneticFieldVector(2) = 0.0;
		normalizedMagneticFieldVector(3) = 0.0;
	}

	MAG_MEAS_MODEL_EULER_BFL::~MAG_MEAS_MODEL_EULER_BFL()
	{
		
	}

    //--> Set-Methods
	//----------------------------------------------------------	
	void MAG_MEAS_MODEL_EULER_BFL::setNormalizedMagneticFieldVector(ColumnVector normalizedMagneticFieldVector)
	{
		this->normalizedMagneticFieldVector = normalizedMagneticFieldVector;
	}
	//----------------------------------------------------------	
	
	//--> Get-Methods
	//----------------------------------------------------------	
	ColumnVector MAG_MEAS_MODEL_EULER_BFL::getNormalizedMagneticFieldVector()
	{
		return normalizedMagneticFieldVector;
	}
	//----------------------------------------------------------	
	
	//--> Measurement equation y = h(x)
	ColumnVector MAG_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);

		//--> Enhance visibility
		//----------------------------------------------------------	
		double rol		= x(ROLL);
		double pit		= x(PITCH);
		double azi		= x(YAW);

		double bx = normalizedMagneticFieldVector(1);
		double by = normalizedMagneticFieldVector(2);
		double bz = normalizedMagneticFieldVector(3);
		//----------------------------------------------------------
			
		//--> Speed up calculations
		//----------------------------------------------------------
		double sin_rol = sin(rol);
		double sin_pit = sin(pit);
		double sin_azi = sin(azi);
		double cos_rol = cos(rol);
		double cos_pit = cos(pit);
		double cos_azi = cos(azi);
		//----------------------------------------------------------

		ColumnVector y(NUMBER_OF_MEASUREMENTS_MAG);

		y(1) = (cos_pit*cos_azi)*bx+(cos_pit*sin_azi)*by+(-sin_pit)*bz;
		y(2) = (sin_rol*sin_pit*cos_azi-cos_rol*sin_azi)*bx+(sin_rol*sin_pit*sin_azi+cos_rol*cos_azi)*by+(sin_rol*cos_pit)*bz;
		y(3) = (cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*bx+(cos_rol*sin_pit*sin_azi-sin_rol*cos_azi)*by+(cos_rol*cos_pit)*bz;

		return y + AdditiveNoiseMuGet();
	}
	
	//--> Jacobian matrix H
	Matrix MAG_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
	{
		//--> Derivative to the first conditional argument (x)
		if (i==0)
		{
			ColumnVector x = ConditionalArgumentGet(0);

			//--> Enhance visibility
			//----------------------------------------------------------
			double rol	   = x(ROLL);
			double pit	   = x(PITCH);
			double azi	   = x(YAW);

			double bx = normalizedMagneticFieldVector(1);
			double by = normalizedMagneticFieldVector(2);
			double bz = normalizedMagneticFieldVector(3);
			//----------------------------------------------------------
			
			//--> Speed up calculations
			//----------------------------------------------------------
			double sin_rol = sin(rol);
			double sin_pit = sin(pit);
			double sin_azi = sin(azi);
			double cos_rol = cos(rol);
			double cos_pit = cos(pit);
			double cos_azi = cos(azi);
			//----------------------------------------------------------

			//--> Set H-Matrix
			//----------------------------------------------------------
			Matrix H(NUMBER_OF_MEASUREMENTS_MAG,NUMBER_OF_STATES);

			//--> Clear H-Matrix
			H = 0.0;
			
			/*
			H(1,1) = 0.0;
			H(2,1) = (cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*bx+(cos_rol*sin_pit*sin_azi-sin_rol*cos_azi)*by+(cos_rol*cos_pit)*bz;
			H(3,1) = (-sin_rol*sin_pit*cos_azi+cos_rol*sin_azi)*bx+(-sin_rol*sin_pit*sin_azi-cos_rol*cos_azi)*by-(sin_rol*cos_pit)*bz;

			H(1,2) = (-sin_pit*cos_azi)*bx-(sin_pit*sin_azi)*by-(cos_pit)*bz;
			H(2,2) = (sin_rol*cos_pit*cos_azi)*bx+(sin_rol*cos_pit*sin_azi)*by-(sin_rol*sin_pit)*bz;
			H(3,2) = (cos_rol*cos_pit*cos_azi)*bx+(cos_rol*cos_pit*sin_azi)*by-(cos_rol*sin_pit)*bz;
			*/

			H(1,3) = (-cos_pit*sin_azi)*bx+(cos_pit*cos_azi)*by;
			H(2,3) = (-sin_rol*sin_pit*sin_azi-cos_rol*cos_azi)*bx+(sin_rol*sin_pit*cos_azi-cos_rol*sin_azi)*by;
			H(3,3) = (-cos_rol*sin_pit*sin_azi+sin_rol*cos_azi)*bx+(cos_rol*sin_pit*cos_azi+sin_rol*sin_azi)*by;
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
