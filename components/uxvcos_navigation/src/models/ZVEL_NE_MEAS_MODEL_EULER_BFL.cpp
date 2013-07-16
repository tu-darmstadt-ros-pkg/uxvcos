#include "ZVEL_NE_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	ZVEL_NE_MEAS_MODEL_EULER_BFL::ZVEL_NE_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_ZVEL_NE,NUM_COND_ARGUMENTS_MEASUREMENT_ZVEL_NE)
	{
		
	}

	ZVEL_NE_MEAS_MODEL_EULER_BFL::~ZVEL_NE_MEAS_MODEL_EULER_BFL()
	{
		
	}
	
	//--> Measurement equation y = h(x)
	ColumnVector ZVEL_NE_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);
		ColumnVector y(NUMBER_OF_MEASUREMENTS_ZVEL_NE);

		y(1) = x(VX);
		y(2) = x(VY);

		return y + AdditiveNoiseMuGet();
	}
	
	//--> Jacobian matrix H
	Matrix ZVEL_NE_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
	{
		//--> Derivative to the first conditional argument (x)
		if (i==0)
		{
			//ColumnVector x = ConditionalArgumentGet(0);

			//--> Set H-Matrix
			//----------------------------------------------------------
			Matrix H(NUMBER_OF_MEASUREMENTS_ZVEL_NE,NUMBER_OF_STATES);

			//--> Clear H-Matrix
			H = 0.0;

			H(1,VX) = 1.0;
			H(2,VY) = 1.0;
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
