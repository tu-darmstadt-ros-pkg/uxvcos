#include "GYROZ_BIAS_MEAS_MODEL_EULER_BFL.h"


namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	GYROZ_BIAS_MEAS_MODEL_EULER_BFL::GYROZ_BIAS_MEAS_MODEL_EULER_BFL()
	: AnalyticConditionalGaussianAdditiveNoise(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS,NUM_COND_ARGUMENTS_MEASUREMENT_GYROZ_BIAS)
	, H(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS,NUMBER_OF_STATES)
	{
	  //--> Set H-Matrix
	  //----------------------------------------------------------
	  H = 0.0;
	  H(1,BIAS_WZ) = 1.0;
	  //----------------------------------------------------------
	}

	GYROZ_BIAS_MEAS_MODEL_EULER_BFL::~GYROZ_BIAS_MEAS_MODEL_EULER_BFL()
	{

	}

	//--> Measurement equation y = h(x)
	ColumnVector GYROZ_BIAS_MEAS_MODEL_EULER_BFL::ExpectedValueGet() const
	{
		const ColumnVector& x = ConditionalArgumentGet(0);
		return H * x + AdditiveNoiseMuGet();
	}

	//--> Jacobian matrix H
	Matrix GYROZ_BIAS_MEAS_MODEL_EULER_BFL::dfGet(unsigned int i) const
	{
		//--> Derivative to the first conditional argument (x)
		if (i==0)
		{
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
