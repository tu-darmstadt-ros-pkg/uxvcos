#ifndef CONTROLLER_QUADROTOR_H
#define CONTROLLER_QUADROTOR_H

#include <uxvcos/controller/ControllerTask.h>
#include <uxvcos/controller/Controller.h>
#include <uxvcos/controller/RC.h>
#include <uxvcos/controller/PID.h>
#include "Altimeter.h"

#include <uxvcos/filter/PT1.h>
#include <uxvcos/filter/OutlierElimination.h>

#include <nav_msgs/typekit/Odometry.h>
#include <sensor_msgs/typekit/Imu.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

#include <geometry_msgs/typekit/Twist.h>
#include <hector_uav_msgs/typekit/ControllerState.h>
#include <geometry_msgs/typekit/WrenchStamped.h>

#include <hector_uav_msgs/typekit/MotorStatus.h>
#include <hector_uav_msgs/typekit/MotorPWM.h>

#include <geometry_msgs/Vector3.h>
#include <tf/LinearMath/Scalar.h>

namespace uxvcos {
namespace Controller {

typedef hector_uav_msgs::ControllerState::_mode_type Mode;
typedef hector_uav_msgs::ControllerState::_state_type State;

static const RC::SwitchFunctionType SWITCH_CONTROL_MODE = 1;
static const RC::SwitchFunctionType SWITCH_HEIGHT_MODE  = 2;

static const double PWM2N  = 0.0325; // from PWM to N
static const double PWM2NM = 4.6350e-04; // from PWM to Nm
static const double l_m = 0.555 / 2; // moment arm of motors

class Quadrotor : public ControllerTask {
public:
  Quadrotor(const std::string &name = "Controller");
  virtual ~Quadrotor();

  Mode getControlMode() const;
  bool switchControlMode(Mode controlMode, Mode disableMode = 0);

  void setState(State set);
  void clearState(State clear);
  State getState() const;
  bool getState(State mask) const;

  geometry_msgs::WrenchStamped getWrench() const;
  void writeOutput(const hector_uav_msgs::MotorPWM &) const;

  void engage();
  void shutdown();

protected:
  virtual void afterStop();
  virtual bool beforeExecuteHook();
  virtual void afterExecuteHook();
  virtual void resetHook();

protected:
  RTT::InputPort<nav_msgs::Odometry>            portState;
  RTT::InputPort<sensor_msgs::Imu>              portImu;
  RTT::InputPort<unsigned int>                  portControlMode;
  RTT::InputPort<hector_uav_msgs::MotorStatus>  portMotorStatus;
  RTT::InputPort<geometry_msgs::Twist>          portCommandVelocity;
  RTT::OutputPort<hector_uav_msgs::ControllerState> portControllerState;
  RTT::OutputPort<geometry_msgs::Twist>         portCommandVelocityInput;

private:
  unsigned int requestedControlMode;
  bool setControlMode(Mode controlMode, bool force = false);
  State getState_private() const { return getState(); }

public:
  nav_msgs::Odometry state;
  sensor_msgs::Imu imu;
  geometry_msgs::Vector3 velocity;
  geometry_msgs::Vector3 acceleration;
  hector_uav_msgs::MotorStatus motor_status;
  hector_uav_msgs::ControllerState controller_state;
  struct { tfScalar roll, pitch, yaw; } euler;
  boost::shared_ptr<Altimeter> altimeter;

  struct Controllers;
  Controllers *controller;
};

} // namespace Controller
} // namespace uxvcos

#endif // CONTROLLER_QUADROTOR_H
