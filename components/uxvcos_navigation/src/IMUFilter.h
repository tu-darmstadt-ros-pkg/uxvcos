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

#ifndef __IMUFILTER_H__
#define __IMUFILTER_H__

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/filter/extendedkalmanfilter.h>

namespace uxvcos {
namespace Navigation
{
	#define NUMBER_OF_STATES_ACCELX			4
	#define NUMBER_OF_STATES_ACCELY			4
	#define NUMBER_OF_STATES_ACCELZ			4
	#define NUMBER_OF_STATES_GYROX			4
	#define NUMBER_OF_STATES_GYROY			4
	#define NUMBER_OF_STATES_GYROZ			4

	#define NUMBER_OF_MEASUREMENTS_ACCELX	1
	#define NUMBER_OF_MEASUREMENTS_ACCELY	1
	#define NUMBER_OF_MEASUREMENTS_ACCELZ	1
	#define NUMBER_OF_MEASUREMENTS_GYROX	1
	#define NUMBER_OF_MEASUREMENTS_GYROY	1
	#define NUMBER_OF_MEASUREMENTS_GYROZ	1

	class IMUFilter
	{
		public:
			IMUFilter(void);
			virtual ~IMUFilter(void);
			
			MatrixWrapper::ColumnVector Update(double ax,double ay,double az,double wx,double wy,double wz);

		private:
			//--> Variables for filters
			//------------------------------------------------------------------
			MatrixWrapper::ColumnVector x_est_accelX;
			MatrixWrapper::ColumnVector x_est_accelY;
			MatrixWrapper::ColumnVector x_est_accelZ;
			MatrixWrapper::ColumnVector x_est_gyroX;
			MatrixWrapper::ColumnVector x_est_gyroY;
			MatrixWrapper::ColumnVector x_est_gyroZ;
			MatrixWrapper::ColumnVector y_accelX;
			MatrixWrapper::ColumnVector y_accelY;
			MatrixWrapper::ColumnVector y_accelZ;
			MatrixWrapper::ColumnVector y_gyroX;
			MatrixWrapper::ColumnVector y_gyroY;
			MatrixWrapper::ColumnVector y_gyroZ;
			//------------------------------------------------------------------

			//--> System uncertainty
			//------------------------------------------------------------------
			MatrixWrapper::ColumnVector sys_noise_Mu_accelX;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_accelX;
			BFL::Gaussian system_Uncertainty_accelX;

			MatrixWrapper::ColumnVector sys_noise_Mu_accelY;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_accelY;
			BFL::Gaussian system_Uncertainty_accelY;

			MatrixWrapper::ColumnVector sys_noise_Mu_accelZ;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_accelZ;
			BFL::Gaussian system_Uncertainty_accelZ;

			MatrixWrapper::ColumnVector sys_noise_Mu_gyroX;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_gyroX;
			BFL::Gaussian system_Uncertainty_gyroX;

			MatrixWrapper::ColumnVector sys_noise_Mu_gyroY;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_gyroY;
			BFL::Gaussian system_Uncertainty_gyroY;

			MatrixWrapper::ColumnVector sys_noise_Mu_gyroZ;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_gyroZ;
			BFL::Gaussian system_Uncertainty_gyroZ;
			//------------------------------------------------------------------

			//--> System models
			//------------------------------------------------------------------
			MatrixWrapper::Matrix A_accelX;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_accelX;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_accelX;
			
			MatrixWrapper::Matrix A_accelY;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_accelY;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_accelY;

			MatrixWrapper::Matrix A_accelZ;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_accelZ;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_accelZ;

			MatrixWrapper::Matrix A_gyroX;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_gyroX;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_gyroX;

			MatrixWrapper::Matrix A_gyroY;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_gyroY;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_gyroY;

			MatrixWrapper::Matrix A_gyroZ;
			BFL::LinearAnalyticConditionalGaussian sys_pdf_gyroZ;
			BFL::LinearAnalyticSystemModelGaussianUncertainty sys_model_gyroZ;
			//------------------------------------------------------------------

			//--> Measurement uncertainties
			//------------------------------------------------------------------
			MatrixWrapper::ColumnVector meas_noise_Mu_accelX;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_accelX;
			BFL::Gaussian measurement_Uncertainty_accelX;

			MatrixWrapper::ColumnVector meas_noise_Mu_accelY;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_accelY;
			BFL::Gaussian measurement_Uncertainty_accelY;

			MatrixWrapper::ColumnVector meas_noise_Mu_accelZ;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_accelZ;
			BFL::Gaussian measurement_Uncertainty_accelZ;

			MatrixWrapper::ColumnVector meas_noise_Mu_gyroX;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_gyroX;
			BFL::Gaussian measurement_Uncertainty_gyroX;

			MatrixWrapper::ColumnVector meas_noise_Mu_gyroY;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_gyroY;
			BFL::Gaussian measurement_Uncertainty_gyroY;

			MatrixWrapper::ColumnVector meas_noise_Mu_gyroZ;
			MatrixWrapper::SymmetricMatrix meas_noise_Cov_gyroZ;
			BFL::Gaussian measurement_Uncertainty_gyroZ;
			//------------------------------------------------------------------

			//--> Measurement models
			//------------------------------------------------------------------
			MatrixWrapper::Matrix H_accelX;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_accelX;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_accelX;

			MatrixWrapper::Matrix H_accelY;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_accelY;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_accelY;

			MatrixWrapper::Matrix H_accelZ;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_accelZ;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_accelZ;

			MatrixWrapper::Matrix H_gyroX;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_gyroX;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_gyroX;

			MatrixWrapper::Matrix H_gyroY;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_gyroY;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_gyroY;

			MatrixWrapper::Matrix H_gyroZ;
			BFL::LinearAnalyticConditionalGaussian meas_pdf_gyroZ;
			BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_gyroZ;
			//------------------------------------------------------------------

			//--> Initial states for filters
			//------------------------------------------------------------------
			MatrixWrapper::ColumnVector prior_Mu_accelX;
			MatrixWrapper::SymmetricMatrix prior_Cov_accelX;
			BFL::Gaussian prior_cont_accelX;

			MatrixWrapper::ColumnVector prior_Mu_accelY;
			MatrixWrapper::SymmetricMatrix prior_Cov_accelY;
			BFL::Gaussian prior_cont_accelY;

			MatrixWrapper::ColumnVector prior_Mu_accelZ;
			MatrixWrapper::SymmetricMatrix prior_Cov_accelZ;
			BFL::Gaussian prior_cont_accelZ;

			MatrixWrapper::ColumnVector prior_Mu_gyroX;
			MatrixWrapper::SymmetricMatrix prior_Cov_gyroX;
			BFL::Gaussian prior_cont_gyroX;

			MatrixWrapper::ColumnVector prior_Mu_gyroY;
			MatrixWrapper::SymmetricMatrix prior_Cov_gyroY;
			BFL::Gaussian prior_cont_gyroY;

			MatrixWrapper::ColumnVector prior_Mu_gyroZ;
			MatrixWrapper::SymmetricMatrix prior_Cov_gyroZ;
			BFL::Gaussian prior_cont_gyroZ;
			//------------------------------------------------------------------

			//--> Extended KalmanFilters
			//------------------------------------------------------------------
			BFL::ExtendedKalmanFilter *filter_accelX;
			BFL::ExtendedKalmanFilter *filter_accelY;
			BFL::ExtendedKalmanFilter *filter_accelZ;
			BFL::ExtendedKalmanFilter *filter_gyroX;
			BFL::ExtendedKalmanFilter *filter_gyroY;
			BFL::ExtendedKalmanFilter *filter_gyroZ;
			//------------------------------------------------------------------
	};
}
} // namespace uxvcos

#endif
