#ifndef UXVCOS_NAVIGATION_H
#define UXVCOS_NAVIGATION_H

#define CURRENT_YEAR						    	2011

#define INS_STATUS_ALIGNMENT						2
#define INS_STATUS_DEGRADED_NAV						4
#define INS_STATUS_NAV_READY						8


#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/pdf/gaussian.h>
#include <bfl/model/analyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/filter/extendedkalmanfilter.h>

#include <sensor_msgs/typekit/Imu.h>
#include <hector_uav_msgs/typekit/Altimeter.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>
#include <hector_uav_msgs/typekit/Supply.h>
#include <sensor_msgs/typekit/NavSatFix.h>
#include <rosgraph_msgs/typekit/Log.h>

#include <nav_msgs/typekit/Odometry.h>
#include <geometry_msgs/typekit/Twist.h>


#include "Compass.h"
#include "Barometer.h"

#include "IMUFilter.h"
#include "EarthCalculations.h"

#include "models/INS_NED_EULER_BFL.h"
#include "models/BARO_MEAS_MODEL_EULER_BFL.h"
#include "models/GPS_MEAS_MODEL_EULER_BFL.h"
#include "models/MAG_MEAS_MODEL_EULER_BFL.h"
#include "models/FIX_POS_MEAS_MODEL_EULER_BFL.h"
#include "models/ZVEL_NE_MEAS_MODEL_EULER_BFL.h"
#include "models/ZVEL_D_MEAS_MODEL_EULER_BFL.h"
#include "models/EST_AZI_MEAS_MODEL_EULER_BFL.h"
#include "models/GYROZ_BIAS_MEAS_MODEL_EULER_BFL.h"
#include "models/ACCEL_MEAS_MODEL_EULER_BFL.h"

#ifdef HAVE_POSEUPDATE
  #include "PoseUpdate.h"
#endif // ROS_PACKAGE_NAME

namespace uxvcos {
namespace Navigation
{
	class Navigation : public RTT::TaskContext
	{
		protected:
			RTT::InputPort<sensor_msgs::Imu> portIMU;
                        RTT::InputPort<hector_uav_msgs::Altimeter> portBaro;
			RTT::InputPort<geometry_msgs::Vector3Stamped> portMagnetic;
			RTT::InputPort<sensor_msgs::NavSatFix> portGPSPosition;
			RTT::InputPort<geometry_msgs::Vector3Stamped> portGPSVelocity;

      //RTT::OutputPort<Data::Navigation::Solution> portNavigationSolution;
      //RTT::OutputPort<Data::Navigation::Variance> portNavigationVariance;
      RTT::OutputPort<nav_msgs::Odometry> portState;
      RTT::OutputPort<sensor_msgs::Imu> portBiasedIMU;
      RTT::OutputPort<sensor_msgs::NavSatFix> portGlobalPosition;
      //RTT::OutputPort<Data::Navigation::Compass> portCompass;
      //RTT::OutputPort<Data::Navigation::Baro> portAltimeter;
      RTT::OutputPort<rosgraph_msgs::Log> portStatus;

			//RTT::Property<Data::Navigation::ReferencePosition> propertyReference;
			sensor_msgs::NavSatFix Reference;

			bool enableZeroVelocityNE;
			bool enableZeroVelocityD;
			bool enableZeroGyroZ;
			bool enablePoseUpdate;

			virtual bool configureHook();
			virtual void cleanupHook();
			virtual bool startHook();
			virtual void updateHook();
			virtual void stopHook();

		private:
			uxvcos::Time		CurrentTime;
			uxvcos::Time		LastTime;
			double				CycleTime;

			unsigned int		GPSTimer;
			unsigned int		BaroTimer;
			unsigned int		MagTimer;
			unsigned int		GPSTimeout;
			unsigned int		BaroTimeout;
			unsigned int		MagTimeout;

			unsigned int		AlignmentTimer;
			unsigned int		FIX_POS_Timer;
			unsigned int		ZVEL_NE_Timer;
			unsigned int		ZVEL_D_Timer;
			unsigned int		EST_AZI_Timer;
			unsigned int		AlignmentTimeout;
			unsigned int		FIX_POS_Interval;
			unsigned int		ZVEL_NE_Interval;
			unsigned int		ZVEL_D_Interval;
			unsigned int		EST_AZI_Interval;

			double				GPSLastLat;
			double				GPSLastLon;
			double				InitialHeight;
			double				LocalGravity;
			double              RmH,RnH;

			bool				PredictWithIMU;
			bool				UpdateWithBaro;
			bool 				UpdateWithGPS;
			bool 				UpdateWithMag;
			bool				UpdateWithFIX_POS;
			bool				UpdateWithZVEL_NE;
			bool				UpdateWithZVEL_D;
			bool				UpdateWithEST_AZI;

			bool				GPSError;
			bool				BaroError;
			bool				MagError;
			bool                ExternalPoseUpdate;

			bool				SetNewGPSPosition;

			unsigned char		OperatingStatus;

			//------------------------------------------------------------------
			IMUFilter *imuFilter;
			RTT::Property<bool> imuFilterEnabled;
			//------------------------------------------------------------------

			RTT::Property<double> baroMaxFilterError;
      RTT::Property<bool> autoInitializeReferenceAltitude;

			//------------------------------------------------------------------
			Compass                         compass;
			MatrixWrapper::ColumnVector		normalizedMagneticFieldVector;

			TBAROMETER						Barometer;
			TEARTHCALCULATIONS				EarthCalc;
			//------------------------------------------------------------------

			//--> Input variables
			//------------------------------------------------------------------
			sensor_msgs::Imu IMUInput;
                        hector_uav_msgs::Altimeter BaroInput;
			geometry_msgs::Vector3Stamped Magnetic;
			sensor_msgs::NavSatFix GPSPosition;
			geometry_msgs::Vector3Stamped GPSVelocity;

			//--> State variables
			//------------------------------------------------------------------
			nav_msgs::Odometry          NavData;      //!< holding navigation data for external usage
			sensor_msgs::Imu            IMUData;      //!< holding IMU data for external usage
			sensor_msgs::NavSatFix      GlobalData;   //!< holding global position for external usage
			rosgraph_msgs::Log          NavStatus;
			double roll, pitch, yaw;

			//--> Variables for navigation filter
			//------------------------------------------------------------------
			MatrixWrapper::ColumnVector u;
			MatrixWrapper::ColumnVector x_est;
			MatrixWrapper::SymmetricMatrix P_est;
			MatrixWrapper::ColumnVector y_baro;
			MatrixWrapper::ColumnVector y_gps;
			MatrixWrapper::ColumnVector y_mag;
			MatrixWrapper::ColumnVector y_fix_pos;
			MatrixWrapper::ColumnVector y_zvel_ne;
			MatrixWrapper::ColumnVector y_zvel_d;
			MatrixWrapper::ColumnVector y_est_azi;
			MatrixWrapper::ColumnVector y_gyro_z_bias;
			MatrixWrapper::ColumnVector y_accel;
			//------------------------------------------------------------------

			//--> System model
			//------------------------------------------------------------------
			INS_NED_EULER_BFL sys_pdf;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov_ALIGNMENT;
			MatrixWrapper::SymmetricMatrix sys_noise_Cov;
			BFL::AnalyticSystemModelGaussianUncertainty sys_model;
			//------------------------------------------------------------------

			//--> BARO measurement model
			//------------------------------------------------------------------
			BARO_MEAS_MODEL_EULER_BFL baro_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty baro_meas_model;
			//------------------------------------------------------------------

			//--> GPS measurement model
			//------------------------------------------------------------------
			GPS_MEAS_MODEL_EULER_BFL gps_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty gps_meas_model;
			//------------------------------------------------------------------

			//--> MAG measurement model
			//------------------------------------------------------------------
			MAG_MEAS_MODEL_EULER_BFL mag_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty mag_meas_model;
			//------------------------------------------------------------------

			//--> FIX_POS measurement model
			//------------------------------------------------------------------
			FIX_POS_MEAS_MODEL_EULER_BFL fix_pos_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty fix_pos_meas_model;
			//------------------------------------------------------------------

			//--> ZVEL_NE measurement model
			//------------------------------------------------------------------
			ZVEL_NE_MEAS_MODEL_EULER_BFL zvel_ne_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty zvel_ne_meas_model;
			//------------------------------------------------------------------

			//--> ZVEL_D measurement model
			//------------------------------------------------------------------
			ZVEL_D_MEAS_MODEL_EULER_BFL zvel_d_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty zvel_d_meas_model;
			//------------------------------------------------------------------

			//--> EST_AZI measurement model
			//------------------------------------------------------------------
			EST_AZI_MEAS_MODEL_EULER_BFL est_azi_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty est_azi_meas_model;
			//------------------------------------------------------------------

			//--> GYROZ_BIAS measurement model
			//------------------------------------------------------------------
			GYROZ_BIAS_MEAS_MODEL_EULER_BFL gyro_z_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty gyro_z_meas_model;
			//------------------------------------------------------------------

			//--> ACCEL measurement model
			//------------------------------------------------------------------
			ACCEL_MEAS_MODEL_EULER_BFL accel_meas_pdf;
			BFL::AnalyticMeasurementModelGaussianUncertainty accel_meas_model;
			//------------------------------------------------------------------

			//--> Extended KalmanFilter
			//------------------------------------------------------------------
			BFL::Gaussian prior_cont;
			BFL::ExtendedKalmanFilter* pFilter;
			//------------------------------------------------------------------

#ifdef HAVE_POSEUPDATE
		public:
			friend class PoseUpdate;
		private:
			PoseUpdate poseUpdate;
#endif // HAVE_POSEUPDATE

		public:
			Navigation(const std::string &name = "Navigation");
			virtual ~Navigation();

			virtual bool reset();
			virtual void setReferenceAltitude(double altitude);
			virtual void resetAltitude();
			virtual void setReferencePosition(double lat, double lon);
			virtual void resetPosition();
	};
} // namespace Navigation
} // namespace uxvcos

#endif // UXVCOS_NAVIGATION_H
