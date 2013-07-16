#include "ROS.h"

#include <ros_integration/ros_plugin.hpp>
//#include <base/DataPool.h>
#include <base/Application.h>

#include <uxvcos/Time.h>

#include <types/sensors.h>
#include <types/navigation.h>

#include <uxvcos/ControlSource.h>
#include <hector_uav_msgs/typekit/ControllerState.h>
#include <hector_uav_msgs/typekit/Height.h>
#include <nav_msgs/typekit/Odometry.h>
#include <sensor_msgs/typekit/Imu.h>
#include <sensor_msgs/typekit/NavSatFix.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

#include <asctec_msgs/IMUCalcData.h>
#include <asctec_msgs/LLStatus.h>
#include <mav_msgs/Height.h>
#include <gps_common/GPSFix.h>

namespace asctec {
  // **** conversion units
  const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
  const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
  const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
  const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m
}

namespace quadro {

ROS::ROS(const std::string& name)
  : ros_integration::TaskContext(name)
  , listener(this, false)
{
  this->addPublisher<nav_msgs::Odometry>("solution", 10, false);
  this->addPublisher<geometry_msgs::PoseStamped>("pose", 10, false);
  this->addPublisher<geometry_msgs::Vector3Stamped>("velocity", 10, false);
  this->addPublisher<sensor_msgs::Imu>("imu", 10, false);
  this->addPublisher<sensor_msgs::NavSatFix>("global", 10, false);
  this->addPublisher<mav_msgs::Height>("height", 10, false);
  this->addPublisher<mav_msgs::Height>("pressure_height", 10, false);

  // for visualization in ccny_ground_station/ground_station only
  this->addPublisher<asctec_msgs::IMUCalcData>("asctec/IMU_CALCDATA", 10, false);
  this->addPublisher<asctec_msgs::LLStatus>("asctec/LL_STATUS", 10, false);
  this->addPublisher<gps_common::GPSFix>("fix", 10, false);

  this->addAttribute("timeOffset", timeOffset);
}

ROS::~ROS()
{}

bool ROS::startHook() {
  listener.addPort("Supply");

  listener.addPort("Solution");
  listener.addPort("LocalSolution");
  listener.addPort("Compass");
  listener.addPort("Altimeter");
  listener.addPort("GPS");
  listener.addPort("ControlState");
  listener.addPort("UsedHeight");

  return true;
}

void ROS::stopHook() {
  listener.clear();
}

void ROS::updateHook()
{
  RTT::base::AttributeBase *simulationAttribute = ::Application::Instance()->getAttribute("simulation");
  RTT::internal::DataSource<bool> *simulation = simulationAttribute ? RTT::internal::DataSource<bool>::narrow(simulationAttribute->getDataSource().get()) : 0;

//  if (!simulation || (simulation->get() == false)) {
//    timeOffset = uxvcos::Time::now() - Data::Timestamp::getTimestamp();
//  } else {
//    timeOffset = 0.0;
//  }
  timeOffset = 0.0;
  if (startTime.isZero()) startTime = uxvcos::Time::now();

  Data::Navigation::Solution nav;
  Data::Navigation::LocalSolution local;
  Data::Navigation::BiasedIMU imu;
  // ::DataPool::Global()->getLastValue(nav, "Solution");
  // ::DataPool::Global()->getLastValue(local, "LocalSolution");
  // ::DataPool::Global()->getLastValue(imu, "BiasedIMU");
  if ((listener.read("Solution", nav) == RTT::NewData) | (listener.read("LocalSolution", local) == RTT::NewData) | (listener.read("BiasedIMU", imu) == RTT::NewData)) {
    tf::Quaternion orientation; orientation.setRPY(nav.rol, nav.pitch, nav.azimuth);

    nav_msgs::Odometry solution;
    solution.header.stamp = local.getTimestamp() + uxvcos::Duration(timeOffset);
    solution.header.frame_id = "/nav";
    solution.child_frame_id = "/base_link";
    solution.pose.pose.position.x    = local.x;
    solution.pose.pose.position.y    = local.y;
    solution.pose.pose.position.z    = local.z;
    solution.pose.pose.orientation.x = orientation.x();
    solution.pose.pose.orientation.y = -orientation.y();
    solution.pose.pose.orientation.z = -orientation.z();
    solution.pose.pose.orientation.w = orientation.w();
    solution.twist.twist.linear.x    = local.v_x;
    solution.twist.twist.linear.y    = local.v_y;
    solution.twist.twist.linear.z    = local.v_z;
    solution.twist.twist.angular.x   = local.rol_dot;
    solution.twist.twist.angular.y   = local.pitch_dot;
    solution.twist.twist.angular.z   = local.azimuth_dot;

    /*
    static tf::TransformBroadcaster tf_broadcaster;
    transforms.resize(3);

    tf::transformTFToMsg(tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, local.azimuth), tf::Point(local.x, local.y, 0.0)), transforms[0].transform);
    transforms[0].header = solution.header;
    transforms[0].child_frame_id = "/base_footprint";

    tf::transformTFToMsg(tf::Transform(tf::Quaternion::getIdentity(), tf::Point(0.0, 0.0, local.z)), transforms[1].transform);
    transforms[1].header = solution.header;
    transforms[1].header.frame_id = transforms[0].child_frame_id;
    transforms[1].child_frame_id = "/base_stabilized";

    tf::transformTFToMsg(tf::Transform(tf::createQuaternionFromRPY(local.rol, local.pitch, 0.0), tf::Point(0.0, 0.0, 0.0)), transforms[2].transform);
    transforms[2].header = solution.header;
    transforms[2].header.frame_id = transforms[1].child_frame_id;
    transforms[2].child_frame_id = "/base_link";

    tf_broadcaster.sendTransform(transforms);
    */

    if (this->getPublisher("solution"))
      this->getPublisher("solution")->publish(solution);

    if (this->getPublisher("pose")) {
      geometry_msgs::PoseStamped pose;
      pose.header = solution.header;
      pose.pose = solution.pose.pose;
      this->getPublisher("pose")->publish(pose);
    }

    if (this->getPublisher("pose")) {
      geometry_msgs::Vector3Stamped velocity;
      velocity.header = solution.header;
      velocity.vector = solution.twist.twist.linear;
      this->getPublisher("velocity")->publish(velocity);
    }

    if (this->getPublisher("imu")) {
      sensor_msgs::Imu output;

  //    Header header
  //      uint32 seq
  //      time stamp
  //      string frame_id
      output.header.stamp = nav.getTimestamp() + uxvcos::Duration(timeOffset);
      output.header.frame_id = "/imu";

  //    geometry_msgs/Quaternion orientation
  //      float64 x
  //      float64 y
  //      float64 z
  //      float64 w
  //    float64[9] orientation_covariance
      tf::Quaternion corientation(orientation *  tf::Quaternion(0.0, 0.0, 0.0, 1.0));
      output.orientation.x = corientation.x();
      output.orientation.y = corientation.y();
      output.orientation.z = corientation.z();
      output.orientation.w = corientation.w();

  //    geometry_msgs/Vector3 angular_velocity
  //      float64 x
  //      float64 y
  //      float64 z
  //    float64[9] angular_velocity_covariance
      output.angular_velocity.x = imu.gyroX;
      output.angular_velocity.y = -imu.gyroY;
      output.angular_velocity.z = -imu.gyroZ;

  //    geometry_msgs/Vector3 linear_acceleration
  //      float64 x
  //      float64 y
  //      float64 z
  //    float64[9] linear_acceleration_covariance
      output.linear_acceleration.x = imu.accelX;
      output.linear_acceleration.y = -imu.accelY;
      output.linear_acceleration.z = -imu.accelZ;

      this->getPublisher("imu")->publish(output);
    }

    if (this->getPublisher("global")) {
      sensor_msgs::NavSatFix output;

      //    uint8 COVARIANCE_TYPE_UNKNOWN=0
      //    uint8 COVARIANCE_TYPE_APPROXIMATED=1
      //    uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
      //    uint8 COVARIANCE_TYPE_KNOWN=3
      //    Header header
      //      uint32 seq
      //      time stamp
      //      string frame_id
      //    sensor_msgs/NavSatStatus status
      //      int8 STATUS_NO_FIX=-1
      //      int8 STATUS_FIX=0
      //      int8 STATUS_SBAS_FIX=1
      //      int8 STATUS_GBAS_FIX=2
      //      uint16 SERVICE_GPS=1
      //      uint16 SERVICE_GLONASS=2
      //      uint16 SERVICE_COMPASS=4
      //      uint16 SERVICE_GALILEO=8
      //      int8 status
      //      uint16 service
      //    float64 latitude
      //    float64 longitude
      //    float64 altitude
      //    float64[9] position_covariance
      //    uint8 position_covariance_type

      output.header.stamp = nav.getTimestamp() + uxvcos::Duration(timeOffset);
      output.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      output.latitude = nav.lat * 180.0/M_PI;
      output.longitude = nav.lon * 180.0/M_PI;
      output.altitude = nav.altitude;

      this->getPublisher("global")->publish(output);
    }
  }

  quadro_msgs::Height height;
  if (listener.read("height", height) == RTT::NewData && this->getPublisher("height")) {
    mav_msgs::Height output;

    output.header = height.header;
    output.height = height.height;
    output.climb = height.vertical_speed;

    this->getPublisher("height")->publish(output);
  }

  Data::Navigation::Baro baro;
  // ::DataPool::Global()->getLastValue(baro, "Altimeter");
  if (listener.read("Altimeter", baro) == RTT::NewData && this->getPublisher("pressure_height")) {
    mav_msgs::Height output;

    output.header.stamp = baro.getTimestamp() + uxvcos::Duration(timeOffset);
    output.height =  baro.HeightQNH;

    this->getPublisher("pressure_height")->publish(output);
  }

  Data::Navigation::GPS gps;
  // ::DataPool::Global()->getLastValue(gps, "GPS");
  if (listener.read("GPS", gps) == RTT::NewData && this->getPublisher("fix")) {
    gps_common::GPSFix output;

    //    uint8 COVARIANCE_TYPE_UNKNOWN=0
    //    uint8 COVARIANCE_TYPE_APPROXIMATED=1
    //    uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
    //    uint8 COVARIANCE_TYPE_KNOWN=3
    //    Header header
    //        uint32 seq
    //        time stamp
    //        string frame_id
    output.header.stamp = gps.getTimestamp() + uxvcos::Duration(timeOffset);

//    GPSStatus status
//        int16 STATUS_NO_FIX=-1
//        int16 STATUS_FIX=0
//        int16 STATUS_SBAS_FIX=1
//        int16 STATUS_GBAS_FIX=2
//        int16 STATUS_DGPS_FIX=18
//        int16 STATUS_WAAS_FIX=33
//        uint16 SOURCE_NONE=0
//        uint16 SOURCE_GPS=1
//        uint16 SOURCE_POINTS=2
//        uint16 SOURCE_DOPPLER=4
//        uint16 SOURCE_ALTIMETER=8
//        uint16 SOURCE_MAGNETIC=16
//        uint16 SOURCE_GYRO=32
//        uint16 SOURCE_ACCEL=64
//        Header header
//            uint32 seq
//            time stamp
//            string frame_id
//        uint16 satellites_used
//        int32[] satellite_used_prn
//        uint16 satellites_visible
//        int32[] satellite_visible_prn
//        int32[] satellite_visible_z
//        int32[] satellite_visible_azimuth
//        int32[] satellite_visible_snr
//        int16 status
//        uint16 motion_source
//        uint16 orientation_source
//        uint16 position_source
    output.status.header = output.header;
    output.status.satellites_used = gps.numberOfSatellites;
    output.status.status = (gps.signalQuality == 3 ? 0 : -1);

//    float64 latitude
    output.latitude = nav.lat * 180.0/M_PI;
//    float64 longitude
    output.longitude = nav.lon * 180.0/M_PI;
//    float64 altitude
    output.altitude = nav.altitude;
//    float64 track
    output.track = atan2(nav.v_e, nav.v_n) * 180.0/M_PI;
//    float64 speed
    output.speed = sqrt(nav.v_e*nav.v_e + nav.v_n*nav.v_n);
//    float64 climb
    output.climb = -nav.v_d;
//    float64 pitch
    output.pitch = nav.pitch * 180.0/M_PI;
//    float64 roll
    output.roll = nav.rol * 180.0/M_PI;
//    float64 dip
    output.dip = nav.azimuth * 180.0/M_PI;
//    float64 time
    output.time = gps.utc;
//    float64 gdop
//    float64 pdop
    output.pdop = gps.pdop;
//    float64 hdop
//    float64 vdop
//    float64 tdop
//    float64 err
//    float64 err_horz
//    float64 err_vert
//    float64 err_track
//    float64 err_speed
//    float64 err_climb
//    float64 err_time
//    float64 err_pitch
//    float64 err_roll
//    float64 err_dip
//    float64[9] position_covariance
//    uint8 position_covariance_type

    this->getPublisher("fix")->publish(output);
  }

  Data::Sensor::Supply supply;
  // ::DataPool::Global()->getLastValue(supply, "Supply");
  listener.read("Supply", supply);

  Data::Navigation::Compass compass;
  // ::DataPool::Global()->getLastValue(compass, "Compass");
  listener.read("Compass", compass);

  hector_uav_msgs::ControllerState state;
  // ::DataPool::Global()->getLastValue(state, "ControlState");
  listener.read("ControlState", state);

  if (this->getPublisher("asctec/IMU_CALCDATA")) {
    asctec_msgs::IMUCalcData output;
    output.header.stamp = nav.getTimestamp() + uxvcos::Duration(timeOffset);

    // # angles derived by integration of gyro_outputs, drift compensated by data fusion;
    // #-90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
    output.angle_nick = static_cast<asctec_msgs::IMUCalcData::_angle_nick_type>(nav.pitch   / asctec::ASC_TO_ROS_ANGLE + .5);
    output.angle_roll = static_cast<asctec_msgs::IMUCalcData::_angle_roll_type>(nav.rol     / asctec::ASC_TO_ROS_ANGLE + .5);
    output.angle_yaw  = static_cast<asctec_msgs::IMUCalcData::_angle_yaw_type> (nav.azimuth / asctec::ASC_TO_ROS_ANGLE + .5);
    if (output.angle_yaw < 0) output.angle_yaw += 360000;

    // # angular velocities, raw values [16 bit], bias free, in 0.0154 degree/s (=> 64.8 = 1 degree/s)
    output.angvel_nick = static_cast<asctec_msgs::IMUCalcData::_angvel_nick_type>((imu.gyroY + imu.estBiasGyroY) / asctec::ASC_TO_ROS_ANGVEL + .5);
    output.angvel_roll = static_cast<asctec_msgs::IMUCalcData::_angvel_roll_type>((imu.gyroX + imu.estBiasGyroX) / asctec::ASC_TO_ROS_ANGVEL + .5);
    output.angvel_yaw  = static_cast<asctec_msgs::IMUCalcData::_angvel_yaw_type> ((imu.gyroZ + imu.estBiasGyroZ) / asctec::ASC_TO_ROS_ANGVEL + .5);

    // # acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
    output.acc_x_calib = static_cast<asctec_msgs::IMUCalcData::_acc_x_calib_type>((imu.accelX + imu.estBiasAccelX) / asctec::ASC_TO_ROS_ACC + .5);
    output.acc_y_calib = static_cast<asctec_msgs::IMUCalcData::_acc_y_calib_type>((imu.accelY + imu.estBiasAccelY) / asctec::ASC_TO_ROS_ACC + .5);
    output.acc_z_calib = static_cast<asctec_msgs::IMUCalcData::_acc_z_calib_type>((imu.accelZ + imu.estBiasAccelZ) / asctec::ASC_TO_ROS_ACC + .5);

    // # horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
    output.acc_x = static_cast<asctec_msgs::IMUCalcData::_acc_x_type>(imu.accelX / asctec::ASC_TO_ROS_ACC + .5);
    output.acc_y = static_cast<asctec_msgs::IMUCalcData::_acc_y_type>(imu.accelY / asctec::ASC_TO_ROS_ACC + .5);
    output.acc_z = static_cast<asctec_msgs::IMUCalcData::_acc_z_type>(imu.accelZ / asctec::ASC_TO_ROS_ACC + .5);

    // # reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
    output.acc_angle_nick = static_cast<asctec_msgs::IMUCalcData::_acc_angle_nick_type>(atan2(-imu.accelX, imu.accelZ) / asctec::ASC_TO_ROS_ANGLE + .5);
    output.acc_angle_roll = static_cast<asctec_msgs::IMUCalcData::_acc_angle_roll_type>(atan2( imu.accelY, imu.accelZ) / asctec::ASC_TO_ROS_ANGLE + .5);

    // # total acceleration measured (10000 = 1g)
    output.acc_absolute_value = static_cast<asctec_msgs::IMUCalcData::_acc_absolute_value_type>(sqrt(imu.accelX*imu.accelX + imu.accelY*imu.accelY + imu.accelZ*imu.accelZ) / asctec::ASC_TO_ROS_ACC + .5);

    // # magnetic field sensors output, offset free and scaled; units not determined,
    // # as only the direction of the field vector is taken into account
    output.Hx = static_cast<asctec_msgs::IMUCalcData::_Hx_type>(compass.Bx * 1000.0 + .5);
    output.Hy = static_cast<asctec_msgs::IMUCalcData::_Hx_type>(compass.By * 1000.0 + .5);
    output.Hz = static_cast<asctec_msgs::IMUCalcData::_Hx_type>(compass.Bz * 1000.0 + .5);

    // # compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
    output.mag_heading = static_cast<asctec_msgs::IMUCalcData::_mag_heading_type>(compass.MagHeading / asctec::ASC_TO_ROS_ANGLE + .5);
    if (output.mag_heading < 0) output.mag_heading += 360000;

    // # pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown;
    // # used for short-term position stabilization
    output.speed_x = static_cast<asctec_msgs::IMUCalcData::_speed_x_type>(nav.v_n * 1000.0 + .5);
    output.speed_y = static_cast<asctec_msgs::IMUCalcData::_speed_y_type>(nav.v_e * 1000.0 + .5);
    output.speed_z = static_cast<asctec_msgs::IMUCalcData::_speed_z_type>(nav.v_d * 1000.0 + .5);

    // # height in mm (after data fusion)
    output.height = static_cast<asctec_msgs::IMUCalcData::_height_type>(nav.altitude / asctec::ASC_TO_ROS_HEIGHT + .5);

    // # diff. height in mm/s (after data fusion)
    output.dheight = static_cast<asctec_msgs::IMUCalcData::_dheight_type>(-nav.v_d / asctec::ASC_TO_ROS_HEIGHT + .5);

    // # diff. height measured by the pressure sensor [mm/s]
    output.dheight_reference = static_cast<asctec_msgs::IMUCalcData::_dheight_reference_type>(0);

    // # height measured by the pressure sensor [mm]
    output.height_reference = static_cast<asctec_msgs::IMUCalcData::_height_reference_type>(baro.HeightQNH / asctec::ASC_TO_ROS_HEIGHT + .5);

    this->getPublisher("asctec/IMU_CALCDATA")->publish(output);
  }

  if (this->getPublisher("asctec/LL_STATUS")) {
    asctec_msgs::LLStatus output;

//    Header header
//      uint32 seq
//      time stamp
//      string frame_id
//    int16 battery_voltage_1
//    int16 battery_voltage_2
//    int16 status
//    int16 cpu_load
//    int8 compass_enabled
//    int8 chksum_error
//    int8 flying
//    int8 motors_on
//    int16 flightMode
//    int16 up_time

    output.header.stamp = state.header.stamp + uxvcos::Duration(timeOffset);
    output.battery_voltage_1 = static_cast<asctec_msgs::LLStatus::_battery_voltage_1_type>(supply.voltage);
    output.battery_voltage_2 = static_cast<asctec_msgs::LLStatus::_battery_voltage_2_type>(supply.voltage);
    output.status            = static_cast<asctec_msgs::LLStatus::_status_type>(0);
    output.cpu_load          = static_cast<asctec_msgs::LLStatus::_cpu_load_type>(0);
    output.compass_enabled   = static_cast<asctec_msgs::LLStatus::_compass_enabled_type>(compass.getTimestamp() + uxvcos::Duration(1.0) > nav.getTimestamp() ? 1 : 0);
    output.chksum_error      = static_cast<asctec_msgs::LLStatus::_chksum_error_type>(0);
    output.flying            = static_cast<asctec_msgs::LLStatus::_flying_type>(height.mode & quadro_msgs::Height::NAV ? 1 : 0);
    output.motors_on         = static_cast<asctec_msgs::LLStatus::_motors_on_type>((state.state & hector_uav_msgs::ControllerState::MOTORS_RUNNING) ? 1 : 0);
    output.flightMode        = static_cast<asctec_msgs::LLStatus::_flightMode_type>(0);
    output.up_time           = static_cast<asctec_msgs::LLStatus::_up_time_type>((uxvcos::Time::now() - startTime).toSec() + .5);

    this->getPublisher("asctec/LL_STATUS")->publish(output);
  }
}

bool ROS::breakUpdateHook() {
  return true;
}

} // namespace quadro

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( quadro::ROS )
