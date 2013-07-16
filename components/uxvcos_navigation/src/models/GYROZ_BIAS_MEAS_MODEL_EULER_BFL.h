/*
 * GYROZ_BIAS_MEAS_MODEL_EULER_BFL.h
 */

#ifndef _GYROZ_BIAS_MEAS_MODEL_EULER_BFL_
#define _GYROZ_BIAS_MEAS_MODEL_EULER_BFL_

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
	#define NUM_COND_ARGUMENTS_MEASUREMENT_GYROZ_BIAS 1

	#ifndef NUMBER_OF_MEASUREMENTS_GYROZ_BIAS
		#define NUMBER_OF_MEASUREMENTS_GYROZ_BIAS 1
	#endif

	class GYROZ_BIAS_MEAS_MODEL_EULER_BFL : public BFL::AnalyticConditionalGaussianAdditiveNoise
	{
		public:
			GYROZ_BIAS_MEAS_MODEL_EULER_BFL();
			virtual ~GYROZ_BIAS_MEAS_MODEL_EULER_BFL();

			//--> Measurement equation y = h(x)
			virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;

			//--> Jacobian matrix H
			virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

		private:
			MatrixWrapper::Matrix H;
	};
}

#endif
} // namespace uxvocs
