/*
 * FIX_POS_MODEL_EULER_BFL.h
 */

#ifndef _FIX_POS_MODEL_EULER_BFL_
#define _FIX_POS_MODEL_EULER_BFL_

/**
 * @brief First implementation of fix position measurement model with Orocos-BFL
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
	#define NUM_COND_ARGUMENTS_MEASUREMENT_FIX_POS 1
	
	#ifndef NUMBER_OF_MEASUREMENTS_FIX_POS
		#define NUMBER_OF_MEASUREMENTS_FIX_POS 2
	#endif

	class FIX_POS_MEAS_MODEL_EULER_BFL : public BFL::AnalyticConditionalGaussianAdditiveNoise
	{
		public:
			FIX_POS_MEAS_MODEL_EULER_BFL();
			virtual ~FIX_POS_MEAS_MODEL_EULER_BFL();
			
			//--> Measurement equation y = h(x)
			virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
			
			//--> Jacobian matrix H
			virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
	};
}

#endif
} // namespace uxvocs
