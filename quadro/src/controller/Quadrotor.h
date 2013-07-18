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
