#include "Simulink.h"

#include <dlfcn.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <uxvcos/Application.h>
#include <uxvcos/SetupFunction.h>
#include <uxvcos/options/options.h>

#include <rtt/Component.hpp>

namespace uxvcos {

namespace { SetupFunction setup(&quadro::Simulink::setup, "Simulink"); }

namespace Simulink {

template<> struct SimulinkConverter<sensor_msgs::Imu, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, sensor_msgs::Imu &data) {
    if (!simulink.size() == 12) return false;
    data.header.seq = simulink[0];
    data.header.stamp.fromSec(simulink[1]);
    data.orientation.x = simulink[2];
    data.orientation.y = simulink[3];
    data.orientation.z = simulink[4];
    data.orientation.w = simulink[5];
    data.angular_velocity.x = simulink[6];
    data.angular_velocity.y = simulink[7];
    data.angular_velocity.z = simulink[8];
    data.linear_acceleration.x = simulink[9];
    data.linear_acceleration.y = simulink[10];
    data.linear_acceleration.z = simulink[11];
    return true;
  }

  static bool toSimulink(const sensor_msgs::Imu& data, DoubleVector& simulink) {
    simulink.resize(12);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.orientation.x;
    simulink[3]  = data.orientation.y;
    simulink[4]  = data.orientation.z;
    simulink[5]  = data.orientation.w;
    simulink[6]  = data.angular_velocity.x;
    simulink[7]  = data.angular_velocity.y;
    simulink[8]  = data.angular_velocity.z;
    simulink[9]  = data.linear_acceleration.x;
    simulink[10] = data.linear_acceleration.y;
    simulink[11] = data.linear_acceleration.z;
    return true;
  }
};

template<> struct SimulinkConverter<nav_msgs::Odometry, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, nav_msgs::Odometry &data) {
    if (!simulink.size() == 15) return false;
    data.header.seq = simulink[0];
    data.header.stamp.fromSec(simulink[1]);
    data.pose.pose.position.x = simulink[2];
    data.pose.pose.position.y = simulink[3];
    data.pose.pose.position.z = simulink[4];
    data.pose.pose.orientation.x = simulink[5];
    data.pose.pose.orientation.y = simulink[6];
    data.pose.pose.orientation.z = simulink[7];
    data.pose.pose.orientation.w = simulink[8];
    data.twist.twist.linear.x = simulink[9];
    data.twist.twist.linear.y = simulink[10];
    data.twist.twist.linear.z = simulink[11];
    data.twist.twist.angular.x = simulink[12];
    data.twist.twist.angular.y = simulink[13];
    data.twist.twist.angular.z = simulink[14];
    return true;
  }

  static bool toSimulink(const nav_msgs::Odometry& data, DoubleVector& simulink) {
    simulink.resize(15);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.pose.pose.position.x;
    simulink[3]  = data.pose.pose.position.y;
    simulink[4]  = data.pose.pose.position.z;
    simulink[5]  = data.pose.pose.orientation.x;
    simulink[6]  = data.pose.pose.orientation.y;
    simulink[7]  = data.pose.pose.orientation.z;
    simulink[8]  = data.pose.pose.orientation.w;
    simulink[9]  = data.twist.twist.linear.x;
    simulink[10] = data.twist.twist.linear.y;
    simulink[11] = data.twist.twist.linear.z;
    simulink[12] = data.twist.twist.angular.x;
    simulink[13] = data.twist.twist.angular.y;
    simulink[14] = data.twist.twist.angular.z;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::Supply, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::Supply& data, DoubleVector& simulink) {
    simulink.resize(2);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink.insert(simulink.end(), data.voltage.begin(), data.voltage.end());
    simulink.insert(simulink.end(), data.current.begin(), data.current.end());
    return true;
  }
};

template<> struct SimulinkConverter<sensor_msgs::Range, DoubleVector> {
  static bool toSimulink(const sensor_msgs::Range& data, DoubleVector& simulink) {
    simulink.resize(3);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.range;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::Altimeter, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::Altimeter& data, DoubleVector& simulink) {
    simulink.resize(5);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.altitude;
    simulink[3]  = data.pressure;
    simulink[4]  = data.qnh;
    return true;
  }
};

template<> struct SimulinkConverter<hector_std_msgs::Float64, DoubleVector> {
  static bool toSimulink(const hector_std_msgs::Float64& data, DoubleVector& simulink) {
    simulink.resize(3);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.data;
    return true;
  }
};

template<> struct SimulinkConverter<sensor_msgs::NavSatFix, DoubleVector> {
  static bool toSimulink(const sensor_msgs::NavSatFix& data, DoubleVector& simulink) {
    simulink.resize(7);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.status.status;
    simulink[3]  = data.status.service;
    simulink[4]  = data.latitude;
    simulink[5]  = data.longitude;
    simulink[6]  = data.altitude;
    return true;
  }
};

template<> struct SimulinkConverter<geometry_msgs::Vector3Stamped, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, geometry_msgs::Vector3Stamped &data) {
    if (simulink.size() != 5) return false;
    data.header.seq = simulink[0];
    data.header.stamp.fromSec(simulink[1]);
    data.vector.x = simulink[2];
    data.vector.y = simulink[3];
    data.vector.z = simulink[4];
    return true;
  }

  static bool toSimulink(const geometry_msgs::Vector3Stamped& data, DoubleVector& simulink) {
    simulink.resize(5);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.vector.x;
    simulink[3]  = data.vector.y;
    simulink[4]  = data.vector.z;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::PositionXYCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::PositionXYCommand& data, DoubleVector& simulink) {
    simulink.resize(4);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.x;
    simulink[3]  = data.y;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::MotorStatus, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::MotorStatus& data, DoubleVector& simulink) {
    simulink.resize(4);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.on;
    simulink[3]  = data.running;
    simulink.insert(simulink.end(), data.voltage.begin(), data.voltage.end());
    simulink.insert(simulink.end(), data.frequency.begin(), data.frequency.end());
    simulink.insert(simulink.end(), data.current.begin(), data.current.end());
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::VelocityXYCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::VelocityXYCommand& data, DoubleVector& simulink) {
    simulink.resize(4);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.x;
    simulink[3]  = data.y;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::AttitudeCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::AttitudeCommand& data, DoubleVector& simulink) {
    simulink.resize(4);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.roll;
    simulink[3]  = data.pitch;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::HeadingCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::HeadingCommand& data, DoubleVector& simulink) {
    simulink.resize(3);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.heading;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::HeightCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::HeightCommand& data, DoubleVector& simulink) {
    simulink.resize(3);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.height;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::ThrustCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::ThrustCommand& data, DoubleVector& simulink) {
    simulink.resize(3);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.thrust;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::RuddersCommand, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::RuddersCommand& data, DoubleVector& simulink) {
    simulink.resize(5);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.aileron;
    simulink[3]  = data.elevator;
    simulink[4]  = data.rudder;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::ControllerState, DoubleVector> {
  static bool toSimulink(const hector_uav_msgs::ControllerState& data, DoubleVector& simulink) {
    simulink.resize(5);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.source;
    simulink[3]  = data.mode;
    simulink[4]  = data.state;
    return true;
  }
};

template<> struct SimulinkConverter<quadro_msgs::Height, DoubleVector> {
  static bool toSimulink(const quadro_msgs::Height& data, DoubleVector& simulink) {
    simulink.resize(7);
    simulink[0]  = data.header.seq;
    simulink[1]  = data.header.stamp.toSec();
    simulink[2]  = data.mode;
    simulink[3]  = data.height;
    simulink[4]  = data.elevation;
    simulink[5]  = data.vertical_speed;
    simulink[6]  = data.height_above_ground;
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::MotorPWM, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, hector_uav_msgs::MotorPWM &data) {
    if (simulink.size() < 2) return false;
    data.header.seq = simulink[0];
    data.header.stamp = ros::Time(simulink[1]);
    data.pwm.assign(simulink.begin() + 2, simulink.end());
    return true;
  }

  static bool toSimulink(const hector_uav_msgs::MotorPWM &data, DoubleVector &simulink) {
    simulink.resize(2);
    simulink[0] = data.header.seq;
    simulink[1] = data.header.stamp.toSec();
    simulink.insert(simulink.end(), data.pwm.begin(), data.pwm.end());
    return true;
  }
};

template<> struct SimulinkConverter<hector_uav_msgs::ServoCommand, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, hector_uav_msgs::ServoCommand &data) {
    if (simulink.size() < 2) return false;
    data.header.seq = simulink[0];
    data.header.stamp = ros::Time(simulink[1]);
    data.value.assign(simulink.begin() + 2, simulink.end());
    return true;
  }
};

template<> struct SimulinkConverter<hector_std_msgs::Float64Array, DoubleVector> {
  static bool fromSimulink(const DoubleVector &simulink, hector_std_msgs::Float64Array &data) {
    if (simulink.size() < 2) return false;
    data.header.seq = simulink[0];
    data.header.stamp = ros::Time(simulink[1]);
    data.data.assign(simulink.begin() + 2, simulink.end());
    return true;
  }
};

} // namespace Simulink

namespace quadro {

int Simulink::setup() {
  Options::Description options("Simulink options");
  options.add_options()
    ("simulink-navigation", Options::value<std::string>()->implicit_value(std::string()), "Use compiled simulink model for navigation");
  options.add_options()
    ("simulink-controller", Options::value<std::string>()->implicit_value(std::string()), "Use compiled simulink model as controller");
  options.add_options()
    ("simulink-autonomy", Options::value<std::string>()->implicit_value(std::string()), "Use compiled simulink model for autonomy");
  Options::options().add(options);
  return 0;
}

Simulink::Simulink(const std::string &name, const std::string& modelName)
  : RTT::TaskContext(name, RTT::base::TaskCore::PreOperational)
  , SimulinkInterface(this, modelName)

//  , propertyReference(0)
//  , portLog("Log")

  //, switchModeMethod("switchMode", &Simulink::switchMode, this)
{
  this->addInputPort("raw_imu", inputs.portRawIMU);
  this->addInputPort("supply", inputs.portSupply);
  this->addInputPort("sonar_height", inputs.portUltrasound);
  this->addInputPort("altimeter", inputs.portAltimeter);
  this->addInputPort("temperature", inputs.portTemperature);
  this->addInputPort("airspeed", inputs.portAirspeed);
  this->addInputPort("fix", inputs.portGPS);
  this->addInputPort("fix_velocity", inputs.portGPSVelocity);
  this->addInputPort("magnetic", inputs.portMagnetic);
  this->addInputPort("rc", inputs.portRC);
  this->addInputPort("motor_status", inputs.portMotorStatus);

  this->addInputPort("imu", inputs.portIMU).doc("IMU input values");
  this->addInputPort("state", inputs.portState).doc("Navigation solution");

  this->addInputPort("position_command", inputs.portPositionCommand);
  this->addInputPort("velocity_command", inputs.portVelocityCommand);
  this->addInputPort("attitude_command", inputs.portAttitudeCommand);
  this->addInputPort("heading_command", inputs.portHeadingCommand);
  this->addInputPort("height_command", inputs.portHeightCommand);
  this->addInputPort("rudders_command", inputs.portRuddersCommand);
  this->addInputPort("throttle_command", inputs.portThrottleCommand);

  this->addInputPort("controller_state", inputs.portControllerState);
  this->addInputPort("height", inputs.portHeight);
  this->addInputPort("reference_orientation", inputs.portReferenceOrientation);
  this->addInputPort("motor_output", inputs.portMotorOutput);

  this->addOutputPort("imu", outputs.portIMU).doc("IMU output values");
  this->addOutputPort("state", outputs.portState).doc("Navigation solution");
  this->addOutputPort("linear_acceleration_bias", outputs.portLinearAccelerationBias);
  this->addOutputPort("angular_velocity_bias", outputs.portAngularVelocityBias);

  this->addOutputPort("motor_output", outputs.portMotorOutput);
  this->addOutputPort("servo_output", outputs.portServoOutput);
  this->addOutputPort("control_mode", outputs.portControlMode);

  this->addOutputPort("log", outputs.portLogOutput);

  // this->provides()->addOperation( switchModeMethod ).doc("Switches between different test modes").arg("Pitch", "0 = direct, 1 = Pitch, 2 = Speed").arg("Throttle", "0 = direct, 1 = Height").arg("Lateral", "0 = direct, 1 = Roll, 2 = Chi");
  
//  if (::Application::Instance() && ::Application::Instance()->getTask("Navigation")) {
//    propertyReference = ::Application::Instance()->getTask("Navigation")->properties()->getPropertyType<Data::Navigation::ReferencePosition>("ReferencePosition");
//  }
}

Simulink::~Simulink()
{
  stop();
  cleanup();
}
  
bool Simulink::configureHook()
{
  RTT::Logger::In in(getName());

  // load the model
  if (!Options::variables("simulink-controller").empty()) {
    std::string new_model_name = Options::variables<std::string>("simulink-controller");
    if (!new_model_name.empty()) modelName.set(new_model_name);
  }
  if (!SimulinkInterface::configureHook()) return false;

  // connect incoming ports


//  connectToSimulinkPort(portIMU);
//  connectToSimulinkPort(portState);
//  connectToSimulinkPort(portSupply);
//  connectToSimulinkPort(portUltrasound);
//  connectToSimulinkPort(portAltimeter);
//  connectToSimulinkPort(portTemperature);
//  connectToSimulinkPort(portAirspeed);
//  connectToSimulinkPort(portGPS);
//  connectToSimulinkPort(portGPSVelocity);
//  connectToSimulinkPort(portMagnetic);

//  connectToSimulinkPort(portRC);

//  connectToSimulinkPort(portPositionCommand);
//  connectToSimulinkPort(portVelocityCommand);
//  connectToSimulinkPort(portAttitudeCommand);
//  connectToSimulinkPort(portHeadingCommand);
//  connectToSimulinkPort(portHeightCommand);
//  connectToSimulinkPort(portRuddersCommand);
//  connectToSimulinkPort(portThrottleCommand);

//  connectToSimulinkPort(portHeight);
//  connectToSimulinkPort(portReferenceOrientation);

//  // connect outgoing ports
//  connectToSimulinkPort(portMotorOutput);
//  connectToSimulinkPort(portServoOutput);
//  connectToSimulinkPort(portControlMode);

////  connectToSimulinkPort(portLog);

//  if (unconnectedPorts()) return false;

  return true;
}

bool Simulink::startHook()
{
  RTT::Logger::In in(getName());
  return SimulinkInterface::startHook();
}

void Simulink::updateHook()
{
  RTT::Logger::In in(getName());
  SimulinkInterface::updateHook();
}

void Simulink::stopHook()
{
  RTT::Logger::In in(getName());
  SimulinkInterface::stopHook();
}

void Simulink::cleanupHook()
{
  RTT::Logger::In in(getName());
  SimulinkInterface::cleanupHook();
}

ORO_CREATE_COMPONENT( Simulink )

} // namespace quadro
} // namespace uxvcos
