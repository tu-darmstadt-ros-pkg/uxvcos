/*
 * MAG_MEAS_MODEL_EULER_BFL.h
 */

#ifndef _MAG_MEAS_MODEL_EULER_BFL_
#define _MAG_MEAS_MODEL_EULER_BFL_

/**
 * @brief First implementation of magnetometer measurement model with Orocos-BFL
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
	#define NUM_COND_ARGUMENTS_MEASUREMENT_MAG 1
	
	#ifndef NUMBER_OF_MEASUREMENTS_MAG
		#define NUMBER_OF_MEASUREMENTS_MAG 3
	#endif

	class MAG_MEAS_MODEL_EULER_BFL : public BFL::AnalyticConditionalGaussianAdditiveNoise
	{
		public:
			MAG_MEAS_MODEL_EULER_BFL();
			virtual ~MAG_MEAS_MODEL_EULER_BFL();

			//--> Set-Methods
			void setNormalizedMagneticFieldVector(MatrixWrapper::ColumnVector normalizedMagneticFieldVector);
			
			//--> Get-Methods
			MatrixWrapper::ColumnVector getNormalizedMagneticFieldVector();
			
			//--> Measurement equation y = h(x)
			virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
			
			//--> Jacobian matrix H
			virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;

		protected:
			//--> Normalized vector of magnetic field for current position
			MatrixWrapper::ColumnVector normalizedMagneticFieldVector;
	};
}
} // namespace uxvocs

#endif
