#include "Navigation.h"

#include <rtt/Attribute.hpp>
#include <Eigen/Geometry>

#include <rtt/Component.hpp>

template <typename T>
T sqr(T x) { return x*x; }

namespace uxvcos {
namespace Navigation
{
	using namespace BFL;
	using namespace MatrixWrapper;

    Navigation::Navigation(const std::string &name)
    : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
    , portIMU("raw_imu")
    , portBaro("altimeter")
    , portMagnetic("magnetic")
    , portGPSPosition("fix")
    , portGPSVelocity("fix_velocity")
    , portState("state")
    , portBiasedIMU("imu")
    , portGlobalPosition("global")
//    , portCompass("Compass")
//    , portAltimeter("Altimeter")
    , portStatus("navigation_status")

    , imuFilterEnabled("IMUFilterEnabled", "If true, IMU measurements are filtered", false)
    , baroMaxFilterError("BaroMaxFilterError", "Reset altitude if difference between estimated and measured altitude is greater than this value", 0.0)
    , autoInitializeReferenceAltitude("AutoInitializeReferenceAltitude", "Automatically initialize reference altitude with present altitude after alignment", true)

    , compass(this, "Compass")
    , normalizedMagneticFieldVector(3)

	, u(NUMBER_OF_INPUTS)
	, x_est(NUMBER_OF_STATES)
	, P_est(NUMBER_OF_STATES)
	, y_baro(NUMBER_OF_MEASUREMENTS_BARO)
	, y_gps(NUMBER_OF_MEASUREMENTS_GPS)
	, y_mag(NUMBER_OF_MEASUREMENTS_MAG)
	, y_fix_pos(NUMBER_OF_MEASUREMENTS_FIX_POS)
	, y_zvel_ne(NUMBER_OF_MEASUREMENTS_ZVEL_NE)
	, y_zvel_d(NUMBER_OF_MEASUREMENTS_ZVEL_D)
	, y_est_azi(NUMBER_OF_MEASUREMENTS_EST_AZI)
	, y_gyro_z_bias(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS)
	, y_accel(NUMBER_OF_MEASUREMENTS_ACCEL)

	, sys_noise_Cov_ALIGNMENT(NUMBER_OF_STATES)
	, sys_noise_Cov(NUMBER_OF_STATES)

	, sys_model(&sys_pdf)
	, baro_meas_model(&baro_meas_pdf)
	, gps_meas_model(&gps_meas_pdf)
	, mag_meas_model(&mag_meas_pdf)
	, fix_pos_meas_model(&fix_pos_meas_pdf)
	, zvel_ne_meas_model(&zvel_ne_meas_pdf)
	, zvel_d_meas_model(&zvel_d_meas_pdf)
	, est_azi_meas_model(&est_azi_meas_pdf)
	, gyro_z_meas_model(&gyro_z_meas_pdf)
	, accel_meas_model(&accel_meas_pdf)
	, prior_cont(NUMBER_OF_STATES)

#ifdef HAVE_POSEUPDATE
	, poseUpdate(this)
#endif
	{
    this->ports()->addPort(portIMU ).doc("Input for IMU sensor data");
    this->ports()->addPort(portBaro).doc("Input for Baro sensor data");
    this->ports()->addPort(portMagnetic).doc("Input for Magnetic sensor data");
    this->ports()->addPort(portGPSPosition).doc("Input for GPS solution");
    this->ports()->addPort(portGPSVelocity).doc("Input for GPS velocity");

		this->ports()->addPort(portState).doc("Output of navigation solution");
		this->ports()->addPort(portBiasedIMU).doc("Output of IMU values with biases");
		this->ports()->addPort(portGlobalPosition).doc("Output of the global WGS84 position");
//		this->ports()->addPort(portCompass).doc("Output of Compass heading and declination");
//		this->ports()->addPort(portAltimeter).doc("Output of barometric pressure and altitudes");
		this->ports()->addPort(portStatus).doc("Navigation status");

		this->properties()->addProperty("ReferencePosition", Reference).doc("Reference position for local coordinates");
		this->properties()->addProperty(imuFilterEnabled);
		this->properties()->addProperty(baroMaxFilterError);
		this->properties()->addProperty(autoInitializeReferenceAltitude);

		this->attributes()->addAttribute("CycleTime", CycleTime);
		this->attributes()->addAttribute("enableZeroVelocityNE", enableZeroVelocityNE);
		this->attributes()->addAttribute("enableZeroVelocityD", enableZeroVelocityD);
		this->attributes()->addAttribute("enableZeroGyroZ", enableZeroGyroZ);
		this->attributes()->addAttribute("enablePoseUpdate", enablePoseUpdate);

		this->provides()->addOperation("reset", &Navigation::reset, this, RTT::OwnThread);
		this->provides()->addOperation("setReferenceAltitude", &Navigation::setReferenceAltitude, this, RTT::OwnThread);
		this->provides()->addOperation("resetAltitude", &Navigation::resetAltitude, this, RTT::OwnThread);
		this->provides()->addOperation("setReferencePosition", &Navigation::setReferencePosition, this, RTT::OwnThread);
		this->provides()->addOperation("resetPosition", &Navigation::resetPosition, this, RTT::OwnThread);

		enableZeroVelocityNE    = true;
		enableZeroVelocityD     = true;
		enableZeroGyroZ         = true;
		enablePoseUpdate        = true;
	}

	Navigation::~Navigation()
	{
		stop();
		cleanup();
	}

	bool Navigation::configureHook()
	{
		RTT::Logger::In in(getName());

		// reconfigure?
		if (getTaskState() == RTT::TaskContext::Stopped) cleanup();

		CurrentTime = uxvcos::Time();
		LastTime = uxvcos::Time();
		CycleTime = 0;

		//--> Set time intervals for sensors and navigation FSM
		//-------------------------------------------------------
		GPSTimer				= 0;
		BaroTimer				= 0;
		MagTimer				= 0;
		GPSTimeout				= 5*1000;	//ms
		BaroTimeout				= 1*1000;	//ms
		MagTimeout				= 1*1000;	//ms

		AlignmentTimer			= 0;
		FIX_POS_Timer			= 0;
		ZVEL_NE_Timer			= 0;
		ZVEL_D_Timer			= 0;
		EST_AZI_Timer			= 0;
		AlignmentTimeout		= 0; // 15*1000;	//ms
		FIX_POS_Interval		= 0;		//ms
		ZVEL_NE_Interval		= 0;		//ms
		ZVEL_D_Interval			= 0;		//ms
		EST_AZI_Interval		= 0;		//ms
		//-------------------------------------------------------

		//--> Set start position
		//-------------------------------------------------------
		GPSLastLat = 49.86196863;
		GPSLastLon =  8.68275853;
		InitialHeight = 0.0;
		//-------------------------------------------------------

		//--> Set reference position and altitude
		//--------------------------------------------------------------------------------------------------
		if (Reference.latitude == 0.0 && Reference.longitude == 0.0) {
			Reference.latitude = GPSLastLat;
			Reference.longitude = GPSLastLon;
			Reference.altitude = InitialHeight;
		}

		setReferenceAltitude(Reference.altitude);
		setReferencePosition(Reference.latitude, Reference.longitude);

		//--> Initialize flags and operating state
		//-------------------------------------------------------
		PredictWithIMU		= false;

		UpdateWithBaro		= false;
		UpdateWithGPS		= false;
		UpdateWithMag		= false;
		UpdateWithFIX_POS	= false;
		UpdateWithZVEL_NE	= false;
		UpdateWithZVEL_D	= false;
		UpdateWithEST_AZI	= false;

		GPSError			= true;
		BaroError			= true;
		MagError			= true;
		ExternalPoseUpdate              = false;

		SetNewGPSPosition	= true;

		OperatingStatus = INS_STATUS_ALIGNMENT;
		//-------------------------------------------------------

		//--> Initialize navigation filter
		//-------------------------------------------------------

		//--> BARO measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector baro_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_BARO);
		baro_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix baro_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_BARO);
		baro_meas_noise_Cov		  =	   0.0;
		baro_meas_noise_Cov(1,1)  =  sqr(10.0);

		baro_meas_pdf.AdditiveNoiseMuSet(baro_meas_noise_Mu);
		baro_meas_pdf.AdditiveNoiseSigmaSet(baro_meas_noise_Cov);
		//---------------------------------------------------

		//--> GPS measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector gps_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_GPS);
		gps_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix gps_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_GPS);
		gps_meas_noise_Cov		  =	   0.0;
		gps_meas_noise_Cov(1,1)   =   sqr(5.0);
		gps_meas_noise_Cov(2,2)   =   sqr(5.0);
		gps_meas_noise_Cov(3,3)   =   sqr(1.0);
		gps_meas_noise_Cov(4,4)   =   sqr(1.0);

		gps_meas_pdf.AdditiveNoiseMuSet(gps_meas_noise_Mu);
		gps_meas_pdf.AdditiveNoiseSigmaSet(gps_meas_noise_Cov);
		//---------------------------------------------------

		//--> MAG measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector mag_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_MAG);
		mag_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix mag_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_MAG);
		mag_meas_noise_Cov		  =	   0.0;
		mag_meas_noise_Cov(1,1)   =   sqr(5.0 * M_PI/180.0);
		mag_meas_noise_Cov(2,2)   =   sqr(5.0 * M_PI/180.0);
		mag_meas_noise_Cov(3,3)   =   sqr(5.0 * M_PI/180.0);

		mag_meas_pdf.AdditiveNoiseMuSet(mag_meas_noise_Mu);
		mag_meas_pdf.AdditiveNoiseSigmaSet(mag_meas_noise_Cov);
		//---------------------------------------------------

		//--> FIX_POS measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector fix_pos_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_FIX_POS);
		fix_pos_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix fix_pos_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_FIX_POS);
		fix_pos_meas_noise_Cov		  =	   0.0;
		fix_pos_meas_noise_Cov(1,1)   =  sqr(1.0);
		fix_pos_meas_noise_Cov(2,2)   =  sqr(1.0);

		fix_pos_meas_pdf.AdditiveNoiseMuSet(fix_pos_meas_noise_Mu);
		fix_pos_meas_pdf.AdditiveNoiseSigmaSet(fix_pos_meas_noise_Cov);
		//---------------------------------------------------

		//--> ZVEL_NE measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector zvel_ne_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ZVEL_NE);
		zvel_ne_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix zvel_ne_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ZVEL_NE);
		zvel_ne_meas_noise_Cov		  =	   0.0;
		zvel_ne_meas_noise_Cov(1,1)   =  sqr(1.0 * 100.0);
		zvel_ne_meas_noise_Cov(2,2)   =  sqr(1.0 * 100.0);

		zvel_ne_meas_pdf.AdditiveNoiseMuSet(zvel_ne_meas_noise_Mu);
		zvel_ne_meas_pdf.AdditiveNoiseSigmaSet(zvel_ne_meas_noise_Cov);
		//---------------------------------------------------

		//--> ZVEL_D measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector zvel_d_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ZVEL_D);
		zvel_d_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix zvel_d_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ZVEL_D);
		zvel_d_meas_noise_Cov		  =	   0.0;
		zvel_d_meas_noise_Cov(1,1)    =  sqr(1.0 * 100.0);

		zvel_d_meas_pdf.AdditiveNoiseMuSet(zvel_d_meas_noise_Mu);
		zvel_d_meas_pdf.AdditiveNoiseSigmaSet(zvel_d_meas_noise_Cov);
		//---------------------------------------------------

		//--> EST_AZI measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector est_azi_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_EST_AZI);
		est_azi_meas_noise_Mu		  =    0.0;
	
		//--> Covariance
		MatrixWrapper::SymmetricMatrix est_azi_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_EST_AZI);
		est_azi_meas_noise_Cov		  =	   0.0;
		est_azi_meas_noise_Cov(1,1)   =  sqr(5.0 * M_PI/180.0);

		est_azi_meas_pdf.AdditiveNoiseMuSet(est_azi_meas_noise_Mu);
		est_azi_meas_pdf.AdditiveNoiseSigmaSet(est_azi_meas_noise_Cov);
		//---------------------------------------------------

		//--> GYRO_Z measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector gyro_z_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS);
		gyro_z_meas_noise_Mu		  =    0.0;

		//--> Covariance
		MatrixWrapper::SymmetricMatrix gyro_z_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_GYROZ_BIAS);
		gyro_z_meas_noise_Cov		  =	   0.0;
		gyro_z_meas_noise_Cov(1,1)   =   sqr(180.0 * M_PI/180.0);

		gyro_z_meas_pdf.AdditiveNoiseMuSet(gyro_z_meas_noise_Mu);
		gyro_z_meas_pdf.AdditiveNoiseSigmaSet(gyro_z_meas_noise_Cov);
		//---------------------------------------------------

		//--> ACCEL measurement uncertainty R
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector accel_meas_noise_Mu(NUMBER_OF_MEASUREMENTS_ACCEL);
		accel_meas_noise_Mu		  =    0.0;

		//--> Covariance
		MatrixWrapper::SymmetricMatrix accel_meas_noise_Cov(NUMBER_OF_MEASUREMENTS_ACCEL);
		accel_meas_noise_Cov		  =	   0.0;
		accel_meas_noise_Cov(1,1)   =   sqr(1.0);
		accel_meas_noise_Cov(2,2)   =   sqr(1.0);
		accel_meas_noise_Cov(3,3)   =   sqr(1.0);

		accel_meas_pdf.AdditiveNoiseMuSet(accel_meas_noise_Mu);
		accel_meas_pdf.AdditiveNoiseSigmaSet(accel_meas_noise_Cov);
		//---------------------------------------------------


		//--> System uncertainty Q
		//---------------------------------------------------
		//--> Mean
		MatrixWrapper::ColumnVector sys_noise_Mu(NUMBER_OF_STATES);
		sys_noise_Mu		 =    0.0;

		//--> Covariance
		sys_noise_Cov	     =  0.0;
//		sys_noise_Cov(ROLL,ROLL)	         =   1E+1 * 1e-3;
//		sys_noise_Cov(PITCH,PITCH)           =   1E+1 * 1e-3;
//		sys_noise_Cov(YAW,YAW)               =   1E+1 * 1e-3;
//		sys_noise_Cov(PX,PX)                 =   0; // 1E+2 * 1e-3; // 1E-2;
//		sys_noise_Cov(PY,PY)                 =   0; // 1E+2 * 1e-3; // 1.5E-2;
//		sys_noise_Cov(PZ,PZ)                 =   0; // 1E+2 * 1e-3;
//		sys_noise_Cov(VX,VX)                 =   1E+1 * 1e-3;
//		sys_noise_Cov(VY,VY)                 =   1E+1 * 1e-3;
//		sys_noise_Cov(VZ,VZ)                 =   1E+1 * 1e-3;
//		sys_noise_Cov(BIAS_AZ,BIAS_AZ)       =   1E-3 * 1e-3;
//		sys_noise_Cov(BIAS_WX,BIAS_WX)       =   1E-4 * 1e-3;
//		sys_noise_Cov(BIAS_WY,BIAS_WY)       =   1E-4 * 1e-3;
//		sys_noise_Cov(BIAS_WZ,BIAS_WZ)       =   1E-4 * 1e-3;
		sys_noise_Cov(ROLL,ROLL)	         =  sqr(5.0 * M_PI/180.0);
		sys_noise_Cov(PITCH,PITCH)         =  sqr(5.0 * M_PI/180.0);
		sys_noise_Cov(YAW,YAW)             =  sqr(5.0 * M_PI/180.0);
		sys_noise_Cov(PX,PX)               =  sqr(1.0);
		sys_noise_Cov(PY,PY)               =  sqr(1.0);
		sys_noise_Cov(PZ,PZ)               =  sqr(1.0);
		sys_noise_Cov(VX,VX)               =  sqr(1.0);
		sys_noise_Cov(VY,VY)               =  sqr(1.0);
		sys_noise_Cov(VZ,VZ)      	       =  sqr(1.0);
		sys_noise_Cov(BIAS_AZ,BIAS_AZ)     =  sqr(1.0e-6);
		sys_noise_Cov(BIAS_WX,BIAS_WX)     =  sqr(5.0e-6 * M_PI/180.0);
		sys_noise_Cov(BIAS_WY,BIAS_WY)     =  sqr(5.0e-6 * M_PI/180.0);
		sys_noise_Cov(BIAS_WZ,BIAS_WZ)     =  sqr(5.0e-6 * M_PI/180.0);

		sys_noise_Cov_ALIGNMENT = sys_noise_Cov;
		sys_noise_Cov_ALIGNMENT(BIAS_AZ,BIAS_AZ) =  sqr(1.0e-3);
		sys_noise_Cov_ALIGNMENT(BIAS_WX,BIAS_WX) =  sqr(5.0e-3 * M_PI/180.0);
		sys_noise_Cov_ALIGNMENT(BIAS_WY,BIAS_WY) =  sqr(5.0e-3 * M_PI/180.0);
		sys_noise_Cov_ALIGNMENT(BIAS_WZ,BIAS_WZ) =  sqr(5.0e-3 * M_PI/180.0);

		sys_pdf.AdditiveNoiseMuSet(sys_noise_Mu);
		sys_pdf.AdditiveNoiseSigmaSet(sys_noise_Cov_ALIGNMENT);
		sys_pdf.setEarthData(RmH, RnH, -LocalGravity);
		//---------------------------------------------------
	
		//-->  Initialize EKF
		//---------------------------------------------------
		//--> Continuous Gaussian prior (for Kalman filter)
		//--> X0

		MatrixWrapper::ColumnVector prior_Mu(NUMBER_OF_STATES);
		prior_Mu = 0.0;
	
		//--> P0
		MatrixWrapper::SymmetricMatrix prior_Cov(NUMBER_OF_STATES);
		prior_Cov		 =    0.0;
//		prior_Cov(ROLL,ROLL)             =   1E-1 * 1e-3;
//		prior_Cov(PITCH,PITCH)		     =   1E-1 * 1e-3;
//		prior_Cov(YAW,YAW)               =   1E-1 * 1e-3;
//		prior_Cov(PX,PX)                 =   1E-3 * 1e-3;
//		prior_Cov(PY,PY)                 =   1E-3 * 1e-3;
//		prior_Cov(PZ,PZ)                 =   1E-3 * 1e-3;
//		prior_Cov(VX,VX)                 =    0.0 * 1e-3;
//		prior_Cov(VY,VY)                 =    0.0 * 1e-3;
//		prior_Cov(VZ,VZ)                 =    0.0 * 1e-3;
//		prior_Cov(BIAS_AZ,BIAS_AZ)       =   5E-1 * 1e-3;
//		prior_Cov(BIAS_WX,BIAS_WX)       =   5E-1 * 1e-3;
//		prior_Cov(BIAS_WY,BIAS_WY)       =   5E-1 * 1e-3;
//		prior_Cov(BIAS_WZ,BIAS_WZ)       =   5E-1 * 1e-3;
		prior_Cov(ROLL,ROLL)		     =  sqr(1e2 * M_PI/180.0);
		prior_Cov(PITCH,PITCH)		   =  sqr(1e2 * M_PI/180.0);
		prior_Cov(YAW,YAW)		       =  sqr(1e2 * M_PI/180.0);
		prior_Cov(PX,PX)             =  sqr(1e3);
		prior_Cov(PY,PY)             =  sqr(1e3);
		prior_Cov(PZ,PZ)             =  sqr(1e3);
		prior_Cov(VX,VX)             =  sqr(1e3);
		prior_Cov(VY,VY)             =  sqr(1e3);
		prior_Cov(VZ,VZ)             =  sqr(1e3);
		prior_Cov(BIAS_AZ,BIAS_AZ)   =  sqr(0.1);
		prior_Cov(BIAS_WX,BIAS_WX)   =  sqr(1.0 * M_PI/180.0);
		prior_Cov(BIAS_WY,BIAS_WY)   =  sqr(1.0 * M_PI/180.0);
		prior_Cov(BIAS_WZ,BIAS_WZ)   =  sqr(1.0 * M_PI/180.0);

		prior_cont.ExpectedValueSet(prior_Mu);
		prior_cont.CovarianceSet(prior_Cov);
		pFilter = new ExtendedKalmanFilter(&prior_cont);
		//---------------------------------------------------

		//--> Initialize IMU-Filter
		imuFilter = new IMUFilter;

		//--> Reset state variables
		x_est = pFilter->PostGet()->ExpectedValueGet();
		P_est = pFilter->PostGet()->CovarianceGet();
		NavData = nav_msgs::Odometry();
		IMUData = sensor_msgs::Imu();
		GlobalData = Reference;
	
		return true;
	}

	void Navigation::cleanupHook()
	{
		RTT::Logger::In in(getName());

		delete pFilter;
		delete imuFilter;
	}

	bool Navigation::startHook()
	{
		RTT::Logger::In in(getName());
		return true;
	}

	void Navigation::stopHook()
	{
		RTT::Logger::In in(getName());
	}

	bool Navigation::reset()
	{
	  stop();
    cleanup();
    if (!configure()) return false;
		if (!start()) return false;
	  return true;
	}

	void Navigation::setReferenceAltitude(double altitude) {
		Reference.altitude = altitude;
	}

	void Navigation::resetAltitude() {
		setReferenceAltitude(GlobalData.altitude);

		x_est = (pFilter->PostGet())->ExpectedValueGet();
		x_est(PZ) = 0.0;
		pFilter->PostGet()->ExpectedValueSet(x_est);
	}

	void Navigation::setReferencePosition(double lat, double lon) {
		Reference.latitude = lat;
		Reference.longitude = lon;

	  //--> Calculate Earth data
	  //--------------------------------------------------------------------------------------------------
		EarthCalc.CalcLocalGravity(Reference.latitude * M_PI/180.0, Reference.altitude, &LocalGravity);
		EarthCalc.CalcLocalRadii(Reference.latitude * M_PI/180.0, Reference.altitude, &RmH, &RnH);
	  //--------------------------------------------------------------------------------------------------

	  //--> Get magnetic declination and field vector from magnetic world model
		compass.magnetometer().GetDeclination(Reference.latitude * M_PI/180.0,Reference.longitude * M_PI/180.0,0.0,CURRENT_YEAR);
	  compass.magnetometer().GetNormalizedMagnetFieldVector(&normalizedMagneticFieldVector(1),
												    &normalizedMagneticFieldVector(2),
												    &normalizedMagneticFieldVector(3));

		//--> Set normalized magnetic field vector from world magnetic model (model is NED)
		normalizedMagneticFieldVector(2) = -normalizedMagneticFieldVector(2);
		normalizedMagneticFieldVector(3) = -normalizedMagneticFieldVector(3);
		mag_meas_pdf.setNormalizedMagneticFieldVector(normalizedMagneticFieldVector);
	  //-------------------------------------------------------
	}

	void Navigation::resetPosition() {
		setReferencePosition(GlobalData.latitude, GlobalData.longitude);

		x_est = (pFilter->PostGet())->ExpectedValueGet();
		x_est(PX) = 0.0;
		x_est(PY) = 0.0;
		pFilter->PostGet()->ExpectedValueSet(x_est);
	}

	void Navigation::updateHook()
	{
		CycleTime = 0.0;

		//--> Load inertial data
		//--------------------------------------------------------------------------------------------------
		if (portIMU.read(IMUInput) == RTT::NewData) {

			//--> Charge u vector with inertial data
			//--------------------------------------
			if (imuFilterEnabled.get())
			{
				u = imuFilter->Update(IMUInput.linear_acceleration.x,IMUInput.linear_acceleration.y,IMUInput.linear_acceleration.z,IMUInput.angular_velocity.x,IMUInput.angular_velocity.y,IMUInput.angular_velocity.z);
			}
			else
			{
				u(AX) = IMUInput.linear_acceleration.x;
				u(AY) = IMUInput.linear_acceleration.y;
				u(AZ) = IMUInput.linear_acceleration.z;
				u(WX) = IMUInput.angular_velocity.x;
				u(WY) = IMUInput.angular_velocity.y;
				u(WZ) = IMUInput.angular_velocity.z;
			}
			//--------------------------------------

			/*//--> Synthetic IMU data generator
			//--------------------------------------
			u(AX) = 0.0;
			u(AY) = 0.0;
			u(AZ) = LocalGravity;
			u(WX) = 0.0;
			u(WY) = 0.0;
			u(WZ) = 0.0;

			static uxvcos::Time StartTime;
			if (OperatingStatus == INS_STATUS_ALIGNMENT) {
				StartTime = CurrentTime;
			} else if (OperatingStatus != INS_STATUS_ALIGNMENT) {
				u(AX) = cos(2.0 * M_PI * 0.1 * (CurrentTime - StartTime));
			}
			*/

			//--> Charge IMU-Structure from INS-Interface with inertial data
			//--------------------------------------------------------------
			IMUData.header = IMUInput.header;
			//--------------------------------------------------------------

			y_gyro_z_bias(1) = -u(WZ);
			y_accel(1) = u(AX);
			y_accel(2) = u(AY);
			y_accel(3) = u(AZ);

			CurrentTime = IMUData.header.stamp;
			if (!LastTime.isZero()) CycleTime = (CurrentTime - LastTime).toSec();
			// assert(CycleTime >= 0 && CycleTime < 1.0);

			PredictWithIMU = true;
		}
		else
		{
			//--> If no IMU data available, then stop navigtion update
			return;
		}
		//--------------------------------------------------------------------------------------------------

		//--> Load barometer data
		//--------------------------------------------------------------------------------------------------
		if (portBaro.read(BaroInput) == RTT::NewData) {

			//--> Calculate heights from barometer data
                        if (Barometer.SetMeasurement(BaroInput.pressure)) {
		
				y_baro(1) = Barometer.GetAltitudeWithQNH() - Reference.altitude;

				UpdateWithBaro = true;

				//--> Clear BaroError flag
				BaroError = false;

				//--> Clear BaroTimer
				BaroTimer = 0;

			} else {

				//--> Set BaroError flag due to illegal pressure value
				BaroError = true;
			}
		}
		else
		{
			//--> Check state of barometer
			if (BaroTimer < BaroTimeout)
			{
				BaroTimer += static_cast<unsigned int>(CycleTime * 1000 + .5);
			}
			else
			{
				if (!BaroError) RTT::log(RTT::Warning) << "Baro failed." << RTT::endlog();

				//--> Set BaroError flag
				BaroError = true;
			}
		}
		//--------------------------------------------------------------------------------------------------

		//--> Load gps data
		//--------------------------------------------------------------------------------------------------
		if ((portGPSPosition.read(GPSPosition) == RTT::NewData || portGPSVelocity.read(GPSVelocity) == RTT::NewData)
				&& GPSPosition.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
		{
			// RTT::log( RTT::Debug ) << "new GPS data (t = " << GPSData.getTimestamp() << ")" << RTT::endlog();
			y_gps(1) = (GPSPosition.latitude - Reference.latitude) * M_PI/180.0 * RmH;
			y_gps(2) = -(GPSPosition.longitude - Reference.longitude) * M_PI/180.0 * RnH;
			y_gps(3) = GPSVelocity.vector.x;
			y_gps(4) = GPSVelocity.vector.y;

      GPSLastLat = GPSPosition.latitude;
      GPSLastLon = GPSPosition.longitude;

      UpdateWithGPS = true;

      //--> Clear GPSError flag
      GPSError = false;

      //--> Clear Timer
      GPSTimer = 0;
		}
		else
		{
			//--> Check state of gps
			if (GPSTimer < GPSTimeout)
			{
				GPSTimer += static_cast<unsigned int>(CycleTime * 1000 + .5);
			}
			else
			{
				if (!GPSError) RTT::log(RTT::Warning) << "GPS failed." << RTT::endlog();

				//--> Set GPSError flag
				GPSError = true;
				
				//--> Set next received GPS position
				SetNewGPSPosition = true;
			}
		}
		//--------------------------------------------------------------------------------------------------

		//--> Load magnetometer data
		//--------------------------------------------------------------------------------------------------
		if (portMagnetic.read(Magnetic) == RTT::NewData &&
		    (Magnetic.vector.x != 0.0 && Magnetic.vector.y != 0.0 && Magnetic.vector.z != 0.0)) {

			//--> Correct deviation and calculate compass information from magnetometer data
			compass.SetMeasurement(Magnetic, x_est(ROLL), x_est(PITCH));

			//--> Caclulate norm of measured magnetic field vector
			compass.GetNormalizedMagneticField(y_mag(1), y_mag(2), y_mag(3));
			UpdateWithMag = true;

			//--> Clear MagError flag
			MagError = false;

			//--> Clear MagTimer
			MagTimer = 0;
		}
		else
		{
			//--> Check state of magnetometer
			if (MagTimer < MagTimeout)
			{
				MagTimer += static_cast<unsigned int>(CycleTime * 1000 + .5);
			}
			else
			{
				if (!MagError) RTT::log(RTT::Warning) << "Magnetometer failed." << RTT::endlog();

				//--> Set MagError flag
				MagError = true;
			}
		}
		//--------------------------------------------------------------------------------------------------

		//--> Navigation Control FSM
		//--------------------------------------------------------------------------------------------------
		if (OperatingStatus == INS_STATUS_ALIGNMENT)
			{
				AlignmentTimer += static_cast<unsigned int>(CycleTime * 1000 + .5);
				
				//--> Do alignment
				if (AlignmentTimer < AlignmentTimeout)
				{
					//--> Do fix position updates
					//--------------------------------------------------------------------------
//					if (true)
//					{
//						FIX_POS_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

//						//--> Force fix position updates (lat,lon) at a certain interval
//						//--------------------------------------------------------------------------
//						if (FIX_POS_Timer >= FIX_POS_Interval)
//						{
//							FIX_POS_Timer = 0;
//							UpdateWithFIX_POS = true;
//						}
//						//--------------------------------------------------------------------------
//					}
					//--------------------------------------------------------------------------
					
					//--> Do zero velocity updates
					//--------------------------------------------------------------------------
					//--> Avoid updates with real GPS data during alignment
					UpdateWithGPS = false;
					UpdateWithZVEL_NE = true;
//					if (true)
//					{
//						ZVEL_NE_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

//						//--> Force zero velocity updates (v_North,v_East) at a certain interval
//						//--------------------------------------------------------------------------
//						if (ZVEL_NE_Timer >= ZVEL_NE_Interval)
//						{
//							ZVEL_NE_Timer = 0;
//							UpdateWithZVEL_NE = true;
//						}
//						//--------------------------------------------------------------------------
//					}
					//--------------------------------------------------------------------------

					//--> Check state of baro
					//--------------------------------------------------------------------------
					UpdateWithBaro = false;
					UpdateWithZVEL_D = true;
//					if (true) // (BaroError)
//					{
//						ZVEL_D_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

//						//--> Force zero velocity updates (v_Down) at a certain interval
//						//--------------------------------------------------------------------------
//						if (ZVEL_D_Timer >= ZVEL_D_Interval)
//						{
//							ZVEL_D_Timer = 0;
//							UpdateWithZVEL_D = true;
//						}
//						//--------------------------------------------------------------------------
//					}
//					else
//						ZVEL_D_Timer = 0;
					//--------------------------------------------------------------------------
				
					//--> Check state of magnetometer
					//--------------------------------------------------------------------------
					UpdateWithMag = false;
					UpdateWithEST_AZI = true;
//					if (true) // (MagError)
//					{
//						EST_AZI_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

//						//--> Force update with estimated azimuth at a certain interval
//						//--------------------------------------------------------------------------
//						if (EST_AZI_Timer >= EST_AZI_Interval)
//						{
//							EST_AZI_Timer = 0;
//							UpdateWithEST_AZI = true;
//						}
//						//--------------------------------------------------------------------------
//					}
//					else
//						EST_AZI_Timer = 0;
					//--------------------------------------------------------------------------
				}
				else
				{
					x_est = (pFilter->PostGet())->ExpectedValueGet();

					//--> Set magnetic heading as azimuth after alignment cycle only if magnetometer is ok
					if (!MagError)
					{
						x_est(YAW) = -compass.GetGeoHeading();			      // azi
					}

					if (!BaroError) {
            if (autoInitializeReferenceAltitude.get()) {
              setReferenceAltitude(Barometer.GetAltitudeWithQNH());
              x_est(PZ) = 0.0;
              y_baro(1) = 0.0;
            } else {
              x_est(PZ) = y_baro(1);
            }
					}

					pFilter->PostGet()->ExpectedValueSet(x_est);

					//--> Set new system noise covariance
					sys_pdf.AdditiveNoiseSigmaSet(sys_noise_Cov);
					
					//--> Clear Timer
					AlignmentTimer = 0;
					FIX_POS_Timer = 0;
					ZVEL_NE_Timer = 0;
					ZVEL_D_Timer = 0;
					EST_AZI_Timer = 0;

					//--> Change operating state
					OperatingStatus = INS_STATUS_DEGRADED_NAV;

					RTT::log(RTT::Info) << "Alignment finished." << RTT::endlog();
				}
			}
			
    if (OperatingStatus == INS_STATUS_DEGRADED_NAV)
    {
      //--> Check state of GPS
      //--------------------------------------------------------------------------
      if (UpdateWithGPS)
      {
        //--> Set new GPS postion if GPS data is available after a GPSError
        //--------------------------------------------------------------------------
        if (SetNewGPSPosition)
        {
          x_est = (pFilter->PostGet())->ExpectedValueGet();
          x_est(PX) = 0.0;
          x_est(PY) = 0.0;
          x_est(VX) = GPSVelocity.vector.x;
          x_est(VY) = GPSVelocity.vector.y;
          pFilter->PostGet()->ExpectedValueSet(x_est);

          setReferencePosition(GPSPosition.latitude, GPSPosition.longitude);

          SetNewGPSPosition = false;
        }
        //--------------------------------------------------------------------------
      }
      else
      {
        if (GPSError && enableZeroVelocityNE)
        {
          ZVEL_NE_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

          //--> Force zero velocity updates (v_North,v_East) at a certain interval
          //--------------------------------------------------------------------------
          if (ZVEL_NE_Timer >= ZVEL_NE_Interval)
          {
            ZVEL_NE_Timer = 0;
            UpdateWithZVEL_NE = true;
          }
          //--------------------------------------------------------------------------
        }
        else
          ZVEL_NE_Timer = 0;
      }
      //--------------------------------------------------------------------------

      //--> Check state of baro
      //--------------------------------------------------------------------------
      if (BaroError && enableZeroVelocityD)
      {
        ZVEL_D_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

        //--> Force zero velocity updates (v_Down) at a certain interval
        //--------------------------------------------------------------------------
        if (ZVEL_D_Timer >= ZVEL_D_Interval)
        {
          ZVEL_D_Timer = 0;
          UpdateWithZVEL_D = true;
        }
        //--------------------------------------------------------------------------
      }
      else
        ZVEL_D_Timer = 0;
      //--------------------------------------------------------------------------

      //--> Check state of magnetometer
      //--------------------------------------------------------------------------
      if (MagError && enableZeroGyroZ)
      {
        EST_AZI_Timer += static_cast<unsigned int>(CycleTime * 1000 + .5);

        //--> Force update with estimated azimuth until magnetometer data is available
        //--------------------------------------------------------------------------
        if (EST_AZI_Timer >= EST_AZI_Interval)
        {
          EST_AZI_Timer = 0;
          UpdateWithEST_AZI = true;
        }
        //--------------------------------------------------------------------------
      }
      else
        EST_AZI_Timer = 0;
      //--------------------------------------------------------------------------

      //--> Change operating state from DEGRADED_NAV to NAV_READY if there are no sensor errors
      //--------------------------------------------------------------------------
      if (!(GPSError) && !(BaroError) && !(MagError)) {
        OperatingStatus = INS_STATUS_NAV_READY;
        RTT::log(RTT::Info) << "Navigation ready." << RTT::endlog();
      }

      //--------------------------------------------------------------------------
    }

    if (OperatingStatus == INS_STATUS_NAV_READY)
    {
      //--> Change operating status from NAV_READY to DEGRADED_NAV and start with
      //--> zero velocity updates (v_North,v_East) if GPSError is detected
      //--------------------------------------------------------------------------
      if (GPSError)
      {
        ZVEL_NE_Timer = 0;
        UpdateWithZVEL_NE = true;

        //--> Change operating state
        OperatingStatus = INS_STATUS_DEGRADED_NAV;
        RTT::log(RTT::Info) << "Navigation degraded (no GPS)." << RTT::endlog();
      }
      //--------------------------------------------------------------------------

      //--> Change operating status from NAV_READY to DEGRADED_NAV and start with
      //--> zero velocity updates (v_Down) if BaroError is detected
      //--------------------------------------------------------------------------
      if (BaroError)
      {
        ZVEL_D_Timer = 0;
        UpdateWithZVEL_D = true;

        //--> Change operating state
        OperatingStatus = INS_STATUS_DEGRADED_NAV;
        RTT::log(RTT::Info) << "Navigation degraded (no Baro)." << RTT::endlog();
      }
      //--------------------------------------------------------------------------

      //--> Change operating status from NAV_READY to DEGRADED_NAV and start with
      //--> updates using estimated azimuth if MagError is detected
      //--------------------------------------------------------------------------
      if (MagError)
      {
        EST_AZI_Timer = 0;
        UpdateWithEST_AZI = true;

        //--> Change operating state
        OperatingStatus = INS_STATUS_DEGRADED_NAV;
        RTT::log(RTT::Info) << "Navigation degraded (no Magnetometer)." << RTT::endlog();
      }
      //--------------------------------------------------------------------------
    }

		//--> Remember a-posteriori heading for EST_AZI updates BEFORE time update
		//--------------------------------------------------------------------------------------------------
		x_est = (pFilter->PostGet())->ExpectedValueGet();
		y_est_azi(1) = x_est(YAW);

		//--> Check if we have an external pose update and therefore don't need the zero velocity updates
		//--------------------------------------------------------------------------------------------------
		if (ExternalPoseUpdate) {
			UpdateWithZVEL_NE = false;
			UpdateWithEST_AZI = false;
		}

		//--> Propagation step
		//--------------------------------------------------------------------------------------------------
		if (PredictWithIMU)
		{
			// RTT::log(RTT::RealTime) << "Predict (ZVEL_NE = " << (UpdateWithZVEL_NE ? "true" : "false") << ", ZVEL_D = " << (UpdateWithZVEL_D ? "true" : "false") << ")" << RTT::endlog();

			sys_pdf.enableZVEL_NE(UpdateWithZVEL_NE);
			sys_pdf.enableZVEL_D(UpdateWithZVEL_D);

			sys_pdf.setDt(CycleTime);
			pFilter->Update(&sys_model,u);

			PredictWithIMU = false;
		}
		//--------------------------------------------------------------------------------------------------

		//--> Update filter with measurements
		//--------------------------------------------------------------------------------------------------
#ifdef HAVE_POSEUPDATE
		if (enablePoseUpdate && OperatingStatus != INS_STATUS_ALIGNMENT) {
			poseUpdate.update();
		}
#endif // HAVE_POSEUPDATE

		if (UpdateWithBaro) {
			// RTT::log(RTT::RealTime) << "UpdateWithBaro y = " << y_baro(1) << RTT::endlog();

			x_est = (pFilter->PostGet())->ExpectedValueGet();
			if (baroMaxFilterError > 0.0 && fabs(y_baro(1) - x_est(6)) > baroMaxFilterError) {
				x_est(PZ) = y_baro(1);
				pFilter->PostGet()->ExpectedValueSet(x_est);
			} else {
				pFilter->Update(&baro_meas_model,y_baro);
			}

			UpdateWithBaro = false;
		}

		if (UpdateWithGPS)
		{
			// RTT::log(RTT::RealTime) << "UpdateWithGPS" << RTT::endlog();
			pFilter->Update(&gps_meas_model,y_gps);

//			// use v_down from GPS too
//			y_zvel_d(1) = -GPSData.v_d;
//			pFilter->Update(&zvel_d_meas_model,y_zvel_d);

			UpdateWithGPS = false;
		}

		if (UpdateWithMag)
		{
			// RTT::log(RTT::RealTime) << "UpdateWithMag" << RTT::endlog();
			pFilter->Update(&mag_meas_model,y_mag);
			UpdateWithMag = false;
		}

		if (UpdateWithFIX_POS)
		{
			// RTT::log(RTT::RealTime) << "UpdateWithFIX_POS" << RTT::endlog();
			y_fix_pos(1) = 0.0;
			y_fix_pos(2) = 0.0;
			pFilter->Update(&fix_pos_meas_model,y_fix_pos);
			UpdateWithFIX_POS = false;
		}

		if (UpdateWithZVEL_NE)
		{
			// RTT::log(RTT::RealTime) << "UpdateWithZVEL_NE" << RTT::endlog();
			y_zvel_ne(1) = 0.0;
			y_zvel_ne(2) = 0.0;
			// pFilter->Update(&zvel_ne_meas_model,y_zvel_ne);
			pFilter->Update(&accel_meas_model, y_accel);
			UpdateWithZVEL_NE = false;
		}

		if (UpdateWithZVEL_D)
		{
			// RTT::log(RTT::RealTime) << "UpdateWithZVEL_D" << RTT::endlog();
			y_zvel_d(1) = 0.0;
			// pFilter->Update(&zvel_d_meas_model,y_zvel_d);
			UpdateWithZVEL_D = false;
		}

		if (UpdateWithEST_AZI)
		{
                        // RTT::log(RTT::RealTime) << "UpdateWithEST_AZI" << RTT::endlog();
			// pFilter->Update(&est_azi_meas_model,y_est_azi);
			pFilter->Update(&gyro_z_meas_model,y_gyro_z_bias);

			UpdateWithEST_AZI = false;
		}
		//--------------------------------------------------------------------------------------------------

		//--> Save filtered data and write navigation solution
		//--------------------------------------------------------------------------------------------------
		x_est = pFilter->PostGet()->ExpectedValueGet();
		P_est = pFilter->PostGet()->CovarianceGet();

		Eigen::Quaterniond orientation(Eigen::AngleAxisd(x_est(YAW), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(x_est(PITCH), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(x_est(ROLL), Eigen::Vector3d::UnitX()));
		NavData.header.stamp = IMUData.header.stamp;
		NavData.pose.pose.orientation.w = orientation.w();
		NavData.pose.pose.orientation.x = orientation.x();
		NavData.pose.pose.orientation.y = orientation.y();
		NavData.pose.pose.orientation.z = orientation.z();
		NavData.pose.pose.position.x = x_est(PX);
		NavData.pose.pose.position.y = x_est(PY);
		NavData.pose.pose.position.z = x_est(PZ);
		NavData.twist.twist.linear.x = x_est(VX);
		NavData.twist.twist.linear.y = x_est(VY);
		NavData.twist.twist.linear.z = x_est(VZ);
		NavData.twist.twist.angular.x = u(WX) + x_est(BIAS_WX);
		NavData.twist.twist.angular.y = u(WY) + x_est(BIAS_WY);
		NavData.twist.twist.angular.z = u(WZ) + x_est(BIAS_WZ);
		portState.write(NavData);

		IMUData.orientation = NavData.pose.pose.orientation;
		IMUData.angular_velocity.x = u(WX) + x_est(BIAS_WX);
		IMUData.angular_velocity.y = u(WY) + x_est(BIAS_WY);
		IMUData.angular_velocity.z = u(WZ) + x_est(BIAS_WZ);
		IMUData.linear_acceleration.x = u(AX) + x_est(BIAS_AX);
		IMUData.linear_acceleration.y = u(AY) + x_est(BIAS_AY);
		IMUData.linear_acceleration.z = u(AZ) + x_est(BIAS_AZ);
		portBiasedIMU.write(IMUData);

		GlobalData.header.stamp = NavData.header.stamp;
		GlobalData.latitude  =  x_est(PX) / RmH * 180.0/M_PI + Reference.latitude;
		GlobalData.longitude = -x_est(PY) / RnH * 180.0/M_PI + Reference.longitude;
		GlobalData.altitude  =  x_est(PZ) + Reference.altitude;
		portGlobalPosition.write(GlobalData);

    NavStatus.header.stamp = NavData.header.stamp;
    NavStatus.name = getName();
    switch (OperatingStatus) {
			case INS_STATUS_ALIGNMENT:
				NavStatus.msg = "ALIGNMENT";
				NavStatus.level = rosgraph_msgs::Log::WARN;
				break;

			case INS_STATUS_DEGRADED_NAV:
				NavStatus.msg = "DEGRADED";
				NavStatus.level = rosgraph_msgs::Log::ERROR;
				break;

			case INS_STATUS_NAV_READY:
				NavStatus.msg = "READY";
				NavStatus.level = rosgraph_msgs::Log::INFO;
				break;
		}
		portStatus.write(NavStatus);
		//--------------------------------------------------------------------------------------------------

// 		//--> Write navigation variance
// 		//--------------------------------------------------------------------------------------------------
// 		for(int i = 0; i < NUMBER_OF_STATES; ++i)
// 			for(int j = 0; j < NUMBER_OF_STATES; ++j)
// 				NavVariance.vector[i*NUMBER_OF_STATES+j] = P_est(i+1,j+1);
// 		NavVariance.setTimestamp(NavData);
// 		portNavigationVariance.write(NavVariance);

		//--> Reset cycle
		//--------------------------------------------------------------------------------------------------
		LastTime = CurrentTime;
		//--------------------------------------------------------------------------------------------------
	}

  ORO_CREATE_COMPONENT( Navigation )

} // namespace Navigation
} // namespace uxvcos
