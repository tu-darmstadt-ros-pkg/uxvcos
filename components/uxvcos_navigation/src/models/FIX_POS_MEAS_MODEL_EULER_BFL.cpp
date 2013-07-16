#include "FIX_POS_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	FIX_POS_MEAS_MODEL_EULER_BFL::FIX_POS_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_FIX_POS,NUM_COND_ARGUMENTS_MEASUREMENT_FIX_POS)
	{
		
	}

	FIX_POS_MEAS_MODEL_EULER_BFL::~FIX_POS_MEAS_MODEL_EULER_BFL()
	{
		
	}
	
	//--> Measurement equation y = h(x)
	ColumnVector FIX_POS_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		ColumnVector x = ConditionalArgumentGet(0);

		//--> Enhance visibility
		//----------------------------------------------------------	
		double Lat		= x(PX);
		double Lon		= x(PY);
		//----------------------------------------------------------

		ColumnVector y(NUMBER_OF_MEASUREMENTS_FIX_POS);

		y(1) = Lat;
		y(2) = Lon;

		return y + AdditiveNoiseMuGet();
	}
	
	//--> Jacobian matrix H
	Matrix FIX_POS_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
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
			Matrix H(NUMBER_OF_MEASUREMENTS_FIX_POS,NUMBER_OF_STATES);

			//--> Clear H-Matrix
			H = 0.0;

			H(1,PX) = 1.0;
			H(2,PY) = 1.0;
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
