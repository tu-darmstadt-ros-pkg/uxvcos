#include "IMUFilter.h"

namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

	IMUFilter::IMUFilter()
		: x_est_accelX(NUMBER_OF_STATES_ACCELX)
		, x_est_accelY(NUMBER_OF_STATES_ACCELY)
		, x_est_accelZ(NUMBER_OF_STATES_ACCELZ)
		, x_est_gyroX(NUMBER_OF_STATES_GYROX)
		, x_est_gyroY(NUMBER_OF_STATES_GYROY)
		, x_est_gyroZ(NUMBER_OF_STATES_GYROZ)

		, y_accelX(NUMBER_OF_MEASUREMENTS_ACCELX)
		, y_accelY(NUMBER_OF_MEASUREMENTS_ACCELY)
		, y_accelZ(NUMBER_OF_MEASUREMENTS_ACCELZ)
		, y_gyroX(NUMBER_OF_MEASUREMENTS_GYROX)
		, y_gyroY(NUMBER_OF_MEASUREMENTS_GYROY)
		, y_gyroZ(NUMBER_OF_MEASUREMENTS_GYROZ)

		, sys_noise_Mu_accelX(NUMBER_OF_STATES_ACCELX)
		, sys_noise_Cov_accelX(NUMBER_OF_STATES_ACCELX)
		, system_Uncertainty_accelX(NUMBER_OF_STATES_ACCELX)

		, sys_noise_Mu_accelY(NUMBER_OF_STATES_ACCELY)
		, sys_noise_Cov_accelY(NUMBER_OF_STATES_ACCELY)
		, system_Uncertainty_accelY(NUMBER_OF_STATES_ACCELY)

		, sys_noise_Mu_accelZ(NUMBER_OF_STATES_ACCELZ)
		, sys_noise_Cov_accelZ(NUMBER_OF_STATES_ACCELZ)
		, system_Uncertainty_accelZ(NUMBER_OF_STATES_ACCELZ)

		, sys_noise_Mu_gyroX(NUMBER_OF_STATES_GYROX)
		, sys_noise_Cov_gyroX(NUMBER_OF_STATES_GYROX)
		, system_Uncertainty_gyroX(NUMBER_OF_STATES_GYROX)

		, sys_noise_Mu_gyroY(NUMBER_OF_STATES_GYROY)
		, sys_noise_Cov_gyroY(NUMBER_OF_STATES_GYROY)
		, system_Uncertainty_gyroY(NUMBER_OF_STATES_GYROY)

		, sys_noise_Mu_gyroZ(NUMBER_OF_STATES_GYROZ)
		, sys_noise_Cov_gyroZ(NUMBER_OF_STATES_GYROZ)
		, system_Uncertainty_gyroZ(NUMBER_OF_STATES_GYROZ)
		
		, A_accelX(NUMBER_OF_STATES_ACCELX,NUMBER_OF_STATES_ACCELX)
		, sys_pdf_accelX(A_accelX,system_Uncertainty_accelX)
		, sys_model_accelX(&sys_pdf_accelX)

		, A_accelY(NUMBER_OF_STATES_ACCELY,NUMBER_OF_STATES_ACCELY)
		, sys_pdf_accelY(A_accelY,system_Uncertainty_accelY)
		, sys_model_accelY(&sys_pdf_accelY)

		, A_accelZ(NUMBER_OF_STATES_ACCELZ,NUMBER_OF_STATES_ACCELZ)
		, sys_pdf_accelZ(A_accelZ,system_Uncertainty_accelZ)
		, sys_model_accelZ(&sys_pdf_accelZ)

		, A_gyroX(NUMBER_OF_STATES_GYROX,NUMBER_OF_STATES_GYROX)
		, sys_pdf_gyroX(A_gyroX,system_Uncertainty_gyroX)
		, sys_model_gyroX(&sys_pdf_gyroX)

		, A_gyroY(NUMBER_OF_STATES_GYROY,NUMBER_OF_STATES_GYROY)
		, sys_pdf_gyroY(A_gyroY,system_Uncertainty_gyroY)
		, sys_model_gyroY(&sys_pdf_gyroY)

		, A_gyroZ(NUMBER_OF_STATES_GYROZ,NUMBER_OF_STATES_GYROZ)
		, sys_pdf_gyroZ(A_gyroZ,system_Uncertainty_gyroZ)
		, sys_model_gyroZ(&sys_pdf_gyroZ)

		, meas_noise_Mu_accelX(NUMBER_OF_MEASUREMENTS_ACCELX)
		, meas_noise_Cov_accelX(NUMBER_OF_MEASUREMENTS_ACCELX)
		, measurement_Uncertainty_accelX(NUMBER_OF_MEASUREMENTS_ACCELX)

		, meas_noise_Mu_accelY(NUMBER_OF_MEASUREMENTS_ACCELY)
		, meas_noise_Cov_accelY(NUMBER_OF_MEASUREMENTS_ACCELY)
		, measurement_Uncertainty_accelY(NUMBER_OF_MEASUREMENTS_ACCELY)

		, meas_noise_Mu_accelZ(NUMBER_OF_MEASUREMENTS_ACCELZ)
		, meas_noise_Cov_accelZ(NUMBER_OF_MEASUREMENTS_ACCELZ)
		, measurement_Uncertainty_accelZ(NUMBER_OF_MEASUREMENTS_ACCELZ)

		, meas_noise_Mu_gyroX(NUMBER_OF_MEASUREMENTS_GYROX)
		, meas_noise_Cov_gyroX(NUMBER_OF_MEASUREMENTS_GYROX)
		, measurement_Uncertainty_gyroX(NUMBER_OF_MEASUREMENTS_GYROX)

		, meas_noise_Mu_gyroY(NUMBER_OF_MEASUREMENTS_GYROY)
		, meas_noise_Cov_gyroY(NUMBER_OF_MEASUREMENTS_GYROY)
		, measurement_Uncertainty_gyroY(NUMBER_OF_MEASUREMENTS_GYROY)

		, meas_noise_Mu_gyroZ(NUMBER_OF_MEASUREMENTS_GYROZ)
		, meas_noise_Cov_gyroZ(NUMBER_OF_MEASUREMENTS_GYROZ)
		, measurement_Uncertainty_gyroZ(NUMBER_OF_MEASUREMENTS_GYROZ)

		, H_accelX(NUMBER_OF_MEASUREMENTS_ACCELX,NUMBER_OF_STATES_ACCELX)
		, meas_pdf_accelX(H_accelX,measurement_Uncertainty_accelX)
		, meas_model_accelX(&meas_pdf_accelX)

		, H_accelY(NUMBER_OF_MEASUREMENTS_ACCELY,NUMBER_OF_STATES_ACCELY)
		, meas_pdf_accelY(H_accelY,measurement_Uncertainty_accelY)
		, meas_model_accelY(&meas_pdf_accelY)

		, H_accelZ(NUMBER_OF_MEASUREMENTS_ACCELZ,NUMBER_OF_STATES_ACCELZ)
		, meas_pdf_accelZ(H_accelZ,measurement_Uncertainty_accelZ)
		, meas_model_accelZ(&meas_pdf_accelZ)

		, H_gyroX(NUMBER_OF_MEASUREMENTS_GYROX,NUMBER_OF_STATES_GYROX)
		, meas_pdf_gyroX(H_gyroX,measurement_Uncertainty_gyroX)
		, meas_model_gyroX(&meas_pdf_gyroX)

		, H_gyroY(NUMBER_OF_MEASUREMENTS_GYROY,NUMBER_OF_STATES_GYROY)
		, meas_pdf_gyroY(H_gyroY,measurement_Uncertainty_gyroY)
		, meas_model_gyroY(&meas_pdf_gyroY)

		, H_gyroZ(NUMBER_OF_MEASUREMENTS_GYROZ,NUMBER_OF_STATES_GYROZ)
		, meas_pdf_gyroZ(H_gyroZ,measurement_Uncertainty_gyroZ)
		, meas_model_gyroZ(&meas_pdf_gyroZ)

		, prior_Mu_accelX(NUMBER_OF_STATES_ACCELX)
		, prior_Cov_accelX(NUMBER_OF_STATES_ACCELX)
		, prior_cont_accelX(NUMBER_OF_STATES_ACCELX)

		, prior_Mu_accelY(NUMBER_OF_STATES_ACCELY)
		, prior_Cov_accelY(NUMBER_OF_STATES_ACCELY)
		, prior_cont_accelY(NUMBER_OF_STATES_ACCELY)

		, prior_Mu_accelZ(NUMBER_OF_STATES_ACCELZ)
		, prior_Cov_accelZ(NUMBER_OF_STATES_ACCELZ)
		, prior_cont_accelZ(NUMBER_OF_STATES_ACCELZ)

		, prior_Mu_gyroX(NUMBER_OF_STATES_GYROX)
		, prior_Cov_gyroX(NUMBER_OF_STATES_GYROX)
		, prior_cont_gyroX(NUMBER_OF_STATES_GYROX)

		, prior_Mu_gyroY(NUMBER_OF_STATES_GYROY)
		, prior_Cov_gyroY(NUMBER_OF_STATES_GYROY)
		, prior_cont_gyroY(NUMBER_OF_STATES_GYROY)

		, prior_Mu_gyroZ(NUMBER_OF_STATES_GYROZ)
		, prior_Cov_gyroZ(NUMBER_OF_STATES_GYROZ)
		, prior_cont_gyroZ(NUMBER_OF_STATES_GYROZ)
	{
		//--> ACCELX measurement model H
		//---------------------------------------------------
		H_accelX = 0.0;
		H_accelX(1,1) = 3.136306536833537e+03;
		H_accelX(1,2) = 6.486300685779483e+05;
		H_accelX(1,3) = 6.153314284226105e+04;
		H_accelX(1,4) = 1.459690470583479e+03;

		meas_pdf_accelX.MatrixSet(0,H_accelX);
		//---------------------------------------------------

		//--> ACCELY measurement model H
		//---------------------------------------------------
		H_accelY = 0.0;
		H_accelY(1,1) = 1.767584159011851e+03;
		H_accelY(1,2) = 5.081729887258509e+05;
		H_accelY(1,3) = 5.225620760505542e+04;
		H_accelY(1,4) = 1.343636837784915e+03;

		meas_pdf_accelY.MatrixSet(0,H_accelY);
		//---------------------------------------------------

		//--> ACCELZ measurement model H
		//---------------------------------------------------
		H_accelZ = 0.0;
		H_accelZ(1,1) = 6.806469978977382e+02;
		H_accelZ(1,2) = 4.449297927869986e+04;
		H_accelZ(1,3) = 1.025986887304700e+04;
		H_accelZ(1,4) = 5.925166094510189e+02;

		meas_pdf_accelZ.MatrixSet(0,H_accelZ);
		//---------------------------------------------------

		//--> GYROX measurement model H
		//---------------------------------------------------
		H_gyroX = 0.0;
		H_gyroX(1,1) = 2.254057137565781e+03;
		H_gyroX(1,2) = 1.421964495727011e+05;
		H_gyroX(1,3) = 2.234103289011709e+04;
		H_gyroX(1,4) = 8.786172884980303e+02;

		meas_pdf_gyroX.MatrixSet(0,H_gyroX);
		//---------------------------------------------------

		//--> GYROY measurement model H
		//---------------------------------------------------
		H_gyroY = 0.0;
		H_gyroY(1,1) = 1.904506081933750e+03;
		H_gyroY(1,2) = 1.131773150783995e+05;
		H_gyroY(1,3) = 1.916906287005370e+04;
		H_gyroY(1,4) = 8.128358174413947e+02;

		meas_pdf_gyroY.MatrixSet(0,H_gyroY);
		//---------------------------------------------------

		//--> GYROZ measurement model H
		//---------------------------------------------------
		H_gyroZ = 0.0;
		H_gyroZ(1,1) = 1.608863924516896e+03;
		H_gyroZ(1,2) = 4.202318693539896e+04;
		H_gyroZ(1,3) = 9.900322679793087e+03;
		H_gyroZ(1,4) = 5.857657474810212e+02;

		meas_pdf_gyroZ.MatrixSet(0,H_gyroZ);
		//---------------------------------------------------

		//--> ACCELX measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_accelX		=    0.0;

		//--> Covariance
		meas_noise_Cov_accelX		=	 0.0;
		meas_noise_Cov_accelX(1,1)  =    0.001082211912583;

		meas_pdf_accelX.AdditiveNoiseMuSet(meas_noise_Mu_accelX);
		meas_pdf_accelX.AdditiveNoiseSigmaSet(meas_noise_Cov_accelX);
		//---------------------------------------------------

		//--> ACCELY measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_accelY		=    0.0;

		//--> Covariance
		meas_noise_Cov_accelY		=	 0.0;
		meas_noise_Cov_accelY(1,1)  =    0.001079729071362;

		meas_pdf_accelY.AdditiveNoiseMuSet(meas_noise_Mu_accelY);
		meas_pdf_accelY.AdditiveNoiseSigmaSet(meas_noise_Cov_accelY);
		//---------------------------------------------------

		//--> ACCELZ measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_accelZ		=    0.0;

		//--> Covariance
		meas_noise_Cov_accelZ		=	 0.0;
		meas_noise_Cov_accelZ(1,1)  =    0.001070272793997;

		meas_pdf_accelZ.AdditiveNoiseMuSet(meas_noise_Mu_accelZ);
		meas_pdf_accelZ.AdditiveNoiseSigmaSet(meas_noise_Cov_accelZ);
		//---------------------------------------------------

		//--> GYROX measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_gyroX			=    0.0;

		//--> Covariance
		meas_noise_Cov_gyroX		=	 0.0;
		meas_noise_Cov_gyroX(1,1)   =    0.001080482576244;

		meas_pdf_gyroX.AdditiveNoiseMuSet(meas_noise_Mu_gyroX);
		meas_pdf_gyroX.AdditiveNoiseSigmaSet(meas_noise_Cov_gyroX);
		//---------------------------------------------------

		//--> GYROY measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_gyroY			=    0.0;

		//--> Covariance
		meas_noise_Cov_gyroY		=	 0.0;
	    meas_noise_Cov_gyroY(1,1)   =    0.001077868288524;

		meas_pdf_gyroY.AdditiveNoiseMuSet(meas_noise_Mu_gyroY);
		meas_pdf_gyroY.AdditiveNoiseSigmaSet(meas_noise_Cov_gyroY);
		//---------------------------------------------------

		//--> GYROZ measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		meas_noise_Mu_gyroZ			=    0.0;

		//--> Covariance
		meas_noise_Cov_gyroZ		=	 0.0;
		meas_noise_Cov_gyroZ(1,1)   =    0.001085531749568;

		meas_pdf_gyroZ.AdditiveNoiseMuSet(meas_noise_Mu_gyroZ);
		meas_pdf_gyroZ.AdditiveNoiseSigmaSet(meas_noise_Cov_gyroZ);
		//---------------------------------------------------

		//--> ACCELX system model A
		//---------------------------------------------------
		A_accelX	  =  0.0;
		A_accelX(1,1) =  1.0;
		A_accelX(1,2) =  0.010000000000000;

		A_accelX(2,2) =  1.0;
		A_accelX(2,3) =  0.010000000000000;
		
		A_accelX(3,3) =  1.0;
		A_accelX(3,4) =  0.010000000000000;

		A_accelX(4,2) =  -3430;
		A_accelX(4,3) =  -147;
		A_accelX(4,4) =  -1.1;

		sys_pdf_accelX.MatrixSet(0,A_accelX);
		//---------------------------------------------------

		//--> ACCELY system model A
		//---------------------------------------------------
		A_accelY	  =  0.0;
		A_accelY(1,1) =  1.0;
		A_accelY(1,2) =  0.010000000000000;

		A_accelY(2,2) =  1.0;
		A_accelY(2,3) =  0.010000000000000;
		
		A_accelY(3,3) =  1.0;
		A_accelY(3,4) =  0.010000000000000;

		A_accelY(4,2) =  -3430;
		A_accelY(4,3) =  -147;
		A_accelY(4,4) =  -1.1;

		sys_pdf_accelY.MatrixSet(0,A_accelY);
		//---------------------------------------------------

		//--> ACCELZ system model A
		//---------------------------------------------------
		A_accelZ	  =  0.0;
		A_accelZ(1,1) =  1.0;
		A_accelZ(1,2) =  0.010000000000000;

		A_accelZ(2,2) =  1.0;
		A_accelZ(2,3) =  0.010000000000000;
		
		A_accelZ(3,3) =  1.0;
		A_accelZ(3,4) =  0.010000000000000;

		A_accelZ(4,2) =  -3430;
		A_accelZ(4,3) =  -147;
		A_accelZ(4,4) =  -1.1;

		sys_pdf_accelZ.MatrixSet(0,A_accelZ);
		//---------------------------------------------------

		//--> GYROX system model A
		//---------------------------------------------------
		A_gyroX	  =  0.0;
		A_gyroX(1,1) =  1.0;
		A_gyroX(1,2) =  0.010000000000000;

		A_gyroX(2,2) =  1.0;
		A_gyroX(2,3) =  0.010000000000000;
		
		A_gyroX(3,3) =  1.0;
		A_gyroX(3,4) =  0.010000000000000;

		A_gyroX(4,2) =  -3430;
		A_gyroX(4,3) =  -147;
		A_gyroX(4,4) =  -1.1;

		sys_pdf_gyroX.MatrixSet(0,A_gyroX);
		//---------------------------------------------------

		//--> GYROY system model A
		//---------------------------------------------------
		A_gyroY	  =  0.0;
		A_gyroY(1,1) =  1.0;
		A_gyroY(1,2) =  0.010000000000000;

		A_gyroY(2,2) =  1.0;
		A_gyroY(2,3) =  0.010000000000000;
		
		A_gyroY(3,3) =  1.0;
		A_gyroY(3,4) =  0.010000000000000;

		A_gyroY(4,2) =  -3430;
		A_gyroY(4,3) =  -147;
		A_gyroY(4,4) =  -1.1;

		sys_pdf_gyroY.MatrixSet(0,A_gyroY);
		//---------------------------------------------------

		//--> GYROZ system model A
		//---------------------------------------------------
		A_gyroZ	  =  0.0;
		A_gyroZ(1,1) =  1.0;
		A_gyroZ(1,2) =  0.010000000000000;

		A_gyroZ(2,2) =  1.0;
		A_gyroZ(2,3) =  0.010000000000000;
		
		A_gyroZ(3,3) =  1.0;
		A_gyroZ(3,4) =  0.010000000000000;

		A_gyroZ(4,2) =  -3430;
		A_gyroZ(4,3) =  -147;
		A_gyroZ(4,4) =  -1.1;

		sys_pdf_gyroZ.MatrixSet(0,A_gyroZ);
		//---------------------------------------------------

		//--> ACCELX system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_accelX			=    0.0;

		//--> Covariance
		sys_noise_Cov_accelX		=    0.0;
		sys_noise_Cov_accelX(1,1)	=    10.0;
		sys_noise_Cov_accelX(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_accelX(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_accelX(4,4)	=    8.666590084232430e-04;

		sys_pdf_accelX.AdditiveNoiseMuSet(sys_noise_Mu_accelX);
		sys_pdf_accelX.AdditiveNoiseSigmaSet(sys_noise_Cov_accelX);
		//---------------------------------------------------

		//--> ACCELY system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_accelY			=    0.0;

		//--> Covariance
		sys_noise_Cov_accelY		=    0.0;
		sys_noise_Cov_accelY(1,1)	=    10.0;
		sys_noise_Cov_accelY(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_accelY(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_accelY(4,4)	=    8.666590084232430e-04;

		sys_pdf_accelY.AdditiveNoiseMuSet(sys_noise_Mu_accelY);
		sys_pdf_accelY.AdditiveNoiseSigmaSet(sys_noise_Cov_accelY);
		//---------------------------------------------------

		//--> ACCELZ system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_accelZ			=    0.0;

		//--> Covariance
		sys_noise_Cov_accelZ		=    0.0;
		sys_noise_Cov_accelZ(1,1)	=    10.0;
		sys_noise_Cov_accelZ(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_accelZ(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_accelZ(4,4)	=    8.666590084232430e-04;

		sys_pdf_accelZ.AdditiveNoiseMuSet(sys_noise_Mu_accelZ);
		sys_pdf_accelZ.AdditiveNoiseSigmaSet(sys_noise_Cov_accelZ);
		//---------------------------------------------------

		//--> GYROX system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_gyroX			=    0.0;

		//--> Covariance
		sys_noise_Cov_gyroX			=    0.0;
		sys_noise_Cov_gyroX(1,1)	=    10.0;
		sys_noise_Cov_gyroX(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroX(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroX(4,4)	=    8.666590084232430e-04;

		sys_pdf_gyroX.AdditiveNoiseMuSet(sys_noise_Mu_gyroX);
		sys_pdf_gyroX.AdditiveNoiseSigmaSet(sys_noise_Cov_gyroX);
		//---------------------------------------------------

		//--> GYROY system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_gyroY			=    0.0;

		//--> Covariance
		sys_noise_Cov_gyroY			=    0.0;
		sys_noise_Cov_gyroY(1,1)	=    10.0;
		sys_noise_Cov_gyroY(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroY(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroY(4,4)	=    8.666590084232430e-04;

		sys_pdf_gyroY.AdditiveNoiseMuSet(sys_noise_Mu_gyroY);
		sys_pdf_gyroY.AdditiveNoiseSigmaSet(sys_noise_Cov_gyroY);
		//---------------------------------------------------

		//--> GYROZ system uncertainty Q
		//---------------------------------------------------
		//--> Mean
		sys_noise_Mu_gyroZ			=    0.0;

		//--> Covariance
		sys_noise_Cov_gyroZ			=    0.0;
		sys_noise_Cov_gyroZ(1,1)	=    10.0;
		sys_noise_Cov_gyroZ(2,2)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroZ(3,3)	=    8.666590084232430e-04;
		sys_noise_Cov_gyroZ(4,4)	=    8.666590084232430e-04;

		sys_pdf_gyroZ.AdditiveNoiseMuSet(sys_noise_Mu_gyroZ);
		sys_pdf_gyroZ.AdditiveNoiseSigmaSet(sys_noise_Cov_gyroZ);
		//---------------------------------------------------

		//-->  Initialize EKF ACCELX
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_accelX				=    0.0;

		//--> P0
		prior_Cov_accelX			=    0.0;
		prior_Cov_accelX(1,1)		=   10.0;
		prior_Cov_accelX(2,2)		=   10.0;
		prior_Cov_accelX(3,3)		=   10.0;
		prior_Cov_accelX(4,4)		=   10.0;

		prior_cont_accelX.ExpectedValueSet(prior_Mu_accelX);
		prior_cont_accelX.CovarianceSet(prior_Cov_accelX);
		filter_accelX = new ExtendedKalmanFilter(&prior_cont_accelX);
		filter_accelX->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_ACCELX);
		//---------------------------------------------------

		//-->  Initialize EKF ACCELY
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_accelY				=    0.0;

		//--> P0
		prior_Cov_accelY			=    0.0;
		prior_Cov_accelY(1,1)		=   10.0;
		prior_Cov_accelY(2,2)		=   10.0;
		prior_Cov_accelY(3,3)		=   10.0;
		prior_Cov_accelY(4,4)		=   10.0;

		prior_cont_accelY.ExpectedValueSet(prior_Mu_accelY);
		prior_cont_accelY.CovarianceSet(prior_Cov_accelY);
		filter_accelY = new ExtendedKalmanFilter(&prior_cont_accelY);
		filter_accelY->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_ACCELY);
		//---------------------------------------------------

		//-->  Initialize EKF ACCELZ
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_accelZ				=    0.0;

		//--> P0
		prior_Cov_accelZ			=    0.0;
		prior_Cov_accelZ(1,1)		=   10.0;
		prior_Cov_accelZ(2,2)		=   10.0;
		prior_Cov_accelZ(3,3)		=   10.0;
		prior_Cov_accelZ(4,4)		=   10.0;

		prior_cont_accelZ.ExpectedValueSet(prior_Mu_accelZ);
		prior_cont_accelZ.CovarianceSet(prior_Cov_accelZ);
		filter_accelZ = new ExtendedKalmanFilter(&prior_cont_accelZ);
		filter_accelZ->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_ACCELZ);
		//---------------------------------------------------

		//-->  Initialize EKF GYROX
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_gyroX				=    0.0;

		//--> P0
		prior_Cov_gyroX				=    0.0;
		prior_Cov_gyroX(1,1)		=   10.0;
		prior_Cov_gyroX(2,2)		=   10.0;
		prior_Cov_gyroX(3,3)		=   10.0;
		prior_Cov_gyroX(4,4)		=   10.0;

		prior_cont_gyroX.ExpectedValueSet(prior_Mu_gyroX);
		prior_cont_gyroX.CovarianceSet(prior_Cov_gyroX);
		filter_gyroX = new ExtendedKalmanFilter(&prior_cont_gyroX);
		filter_gyroX->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_GYROX);
		//---------------------------------------------------

		//-->  Initialize EKF GYROY
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_gyroY				=    0.0;

		//--> P0
		prior_Cov_gyroY				=    0.0;
		prior_Cov_gyroY(1,1)		=   10.0;
		prior_Cov_gyroY(2,2)		=   10.0;
		prior_Cov_gyroY(3,3)		=   10.0;
		prior_Cov_gyroY(4,4)		=   10.0;

		prior_cont_gyroY.ExpectedValueSet(prior_Mu_gyroY);
		prior_cont_gyroY.CovarianceSet(prior_Cov_gyroY);
		filter_gyroY = new ExtendedKalmanFilter(&prior_cont_gyroY);
		filter_gyroY->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_GYROY);
		//---------------------------------------------------

		//-->  Initialize EKF GYROZ
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0
		prior_Mu_gyroZ				=    0.0;

		//--> P0
		prior_Cov_gyroZ				=    0.0;
		prior_Cov_gyroZ(1,1)		=   10.0;
		prior_Cov_gyroZ(2,2)		=   10.0;
		prior_Cov_gyroZ(3,3)		=   10.0;
		prior_Cov_gyroZ(4,4)		=   10.0;

		prior_cont_gyroZ.ExpectedValueSet(prior_Mu_gyroZ);
		prior_cont_gyroZ.CovarianceSet(prior_Cov_gyroZ);
		filter_gyroZ = new ExtendedKalmanFilter(&prior_cont_gyroZ);
		filter_gyroZ->AllocateMeasModelExt(NUMBER_OF_MEASUREMENTS_GYROZ);
		//---------------------------------------------------
	}

	IMUFilter::~IMUFilter()
	{
		delete filter_accelX;
		delete filter_accelY;
		delete filter_accelZ;
		delete filter_gyroX;
		delete filter_gyroY;
		delete filter_gyroZ;
	}

	ColumnVector IMUFilter::Update(double ax,double ay,double az,double wx,double wy,double wz)
	{
		ColumnVector x_est(6);

		y_accelX(1) = ax;
		filter_accelX->Update(&sys_model_accelX,&meas_model_accelX,y_accelX);
		x_est_accelX = filter_accelX->PostGet()->ExpectedValueGet();
		x_est(1) = H_accelX(1,1)*x_est_accelX(1);

		y_accelY(1) = ay;
		filter_accelY->Update(&sys_model_accelY,&meas_model_accelY,y_accelY);
		x_est_accelY = filter_accelY->PostGet()->ExpectedValueGet();
		x_est(2) = H_accelY(1,1)*x_est_accelY(1);

		y_accelZ(1) = az;
		filter_accelZ->Update(&sys_model_accelZ,&meas_model_accelZ,y_accelZ);
		x_est_accelZ = filter_accelZ->PostGet()->ExpectedValueGet();
		x_est(3) = H_accelZ(1,1)*x_est_accelZ(1);

		y_gyroX(1) = wx;
		filter_gyroX->Update(&sys_model_gyroX,&meas_model_gyroX,y_gyroX);
		x_est_gyroX = filter_gyroX->PostGet()->ExpectedValueGet();
		x_est(4) = H_gyroX(1,1)*x_est_gyroX(1);

		y_gyroY(1) = wy;
		filter_gyroY->Update(&sys_model_gyroY,&meas_model_gyroY,y_gyroY);
		x_est_gyroY = filter_gyroY->PostGet()->ExpectedValueGet();
		x_est(5) = H_gyroY(1,1)*x_est_gyroY(1);

		y_gyroZ(1) = wz;
		filter_gyroZ->Update(&sys_model_gyroZ,&meas_model_gyroZ,y_gyroZ);
		x_est_gyroZ = filter_gyroZ->PostGet()->ExpectedValueGet();
		x_est(6) = H_gyroZ(1,1)*x_est_gyroZ(1);

		return x_est;
	}
}
} // namespace uxvcos
