#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

#include <tf/transform_datatypes.h>
#ifdef TF_MATRIX3x3_H
  #define Matrix3x3 tf::Matrix3x3
#else
  #define Matrix3x3 btMatrix3x3
#endif

#include <limits>

#include "Quadrotor.h"
#include "Attitude.h"
#include "Velocity.h"
#include "Position.h"
#include "Heading.h"
#include "Height.h"
#include "Thrust.h"
#include "Force.h"
#include "Motor.h"
#include "Test.h"

#include <controller/wrench_operations.h>

namespace uxvcos {
namespace Controller {

struct Quadrotor::Controllers {
  boost::shared_ptr<Attitude> attitude;
  boost::shared_ptr<Velocity> velocity;
  boost::shared_ptr<Position> position;
  boost::shared_ptr<Heading> heading;
  boost::shared_ptr<Height> height;
  boost::shared_ptr<Thrust> thrust;
  boost::shared_ptr<Force> forces;
  boost::shared_ptr<Motor> motor;
  boost::shared_ptr<Test> test;

  Controllers(Quadrotor *controller)
  {
    attitude = controller->addController(new Attitude(controller), false);
    velocity = controller->addController(new Velocity(controller), false);
    position = controller->addController(new Position(controller), false);
    heading  = controller->addController(new Heading(controller), false);
    height   = controller->addController(new Height(controller), false);
    thrust   = controller->addController(new Thrust(controller), true);
    forces   = controller->addController(new Force(controller), true);
    motor    = controller->addController(new Motor(controller), true);
    test     = controller->addController(new Test(controller), false);
  }
};

Quadrotor::Quadrotor(const std::string &name)
  : ControllerTask(name)
{
  rc = this->addModule(new RC(this));
  altimeter = this->addModule(new Altimeter(this));
  controller = new Controllers(this);

  this->ports()->addPort("state", portState).doc("Current pose and velocity of the vehicle");
  this->ports()->addPort("imu", portImu).doc("IMU input values");
  this->ports()->addPort("control_mode", portControlMode).doc("Sets the control mode of the controller");
  this->ports()->addPort("motor_status", portMotorStatus).doc("Current motor status");
  this->ports()->addPort("controller_state", portControllerState).doc("Internal state of the controller");
  this->ports()->addPort("command_velocity", portCommandVelocity).doc("Command the quadrotor velocities");
  this->ports()->addPort("command_velocity_input", portCommandVelocityInput).doc("Current commanded velocities");

  this->addOperation("switchControlMode", &Quadrotor::switchControlMode, this, RTT::OwnThread).doc("Switches the control mode").arg("mode", "the new mode flags").arg("disable", "mode flags to be disabled");
  this->addOperation("setControlMode", &Quadrotor::setControlMode, this, RTT::OwnThread).doc("Overwrites the control mode").arg("mode", "the new mode").arg("force", "Enforce the new control mode");
  this->addOperation("getControlMode", &Quadrotor::getControlMode, this, RTT::OwnThread).doc("Gets the current control mode");
  this->addOperation("getState", &Quadrotor::getState_private, this, RTT::OwnThread).doc("Gets the current state of the controller");

  setControlSource(CONTROL_AUTONOMOUS);
  setControlMode(hector_uav_msgs::ControllerState::ATTITUDE | hector_uav_msgs::ControllerState::HEADING | hector_uav_msgs::ControllerState::HEIGHT, true);
  requestedControlMode = getControlMode();

  controller->velocity->connect(*(controller->position));
  controller->attitude->connect(*(controller->velocity));
  controller->motor->connect(*(controller->forces));
}

Quadrotor::~Quadrotor() {
  stop();
  cleanup();
  delete controller;
}

void Quadrotor::afterStop() {
}

bool Quadrotor::beforeExecuteHook() {
  if (portState.readNewest(state, false) != RTT::NewData) return false;
  portImu.readNewest(imu, false);
  portMotorStatus.readNewest(motor_status, false);

  // set timestamp and step time dt based on state timestamp
  this->setTimestamp(state.header.stamp);
  RTT::log(RTT::RealTime) << "Updating quadrotor controller with state " << state.header.seq << " at t = " << getTimestamp() << RTT::endlog();

  static std_msgs::Header::_seq_type seq = state.header.seq - 1;
  while (state.header.seq > ++seq) {
    RTT::log(RTT::Warning) << "missed state message " << seq << " at t = " << getTimestamp() << RTT::endlog();
  }

  // read velocity command
  geometry_msgs::Twist command;
  if (portCommandVelocity.readNewest(command, false) == RTT::NewData)
  {
    controller->velocity->setVelocity(command.linear.x, command.linear.y);
    controller->height->setClimbrate(command.linear.z);
    controller->heading->setTurnrate(command.angular.z);

    setControlSource(CONTROL_JOYSTICK);
    switchControlMode(hector_uav_msgs::ControllerState::VELOCITY | hector_uav_msgs::ControllerState::HEIGHT | hector_uav_msgs::ControllerState::HEADING);
  }

  // calculate euler angles
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(state.pose.pose.orientation, quaternion);
  if (quaternion.length2() == 0) return false;
  Matrix3x3(quaternion).getEulerYPR(euler.yaw, euler.pitch, euler.roll);

  // calculate acceleration (nav frame)
  if (!std::isnan(velocity.x) && getDt() > 0) {
    acceleration.x = (state.twist.twist.linear.x - velocity.x) / getDt();
    acceleration.y = (state.twist.twist.linear.y - velocity.y) / getDt();
    acceleration.z = (state.twist.twist.linear.z - velocity.z) / getDt();
  } else {
    acceleration = geometry_msgs::Vector3();
  }
  velocity     = state.twist.twist.linear;

  // switch control mode and source depending on the RC switch
  RC::SwitchType controlModeSwitch;
  if (rc->getSwitch(SWITCH_CONTROL_MODE, controlModeSwitch)) {
    if (controlModeSwitch < 2) {
      if (controlModeSwitch == 0) switchControlMode(hector_uav_msgs::ControllerState::ATTITUDE | hector_uav_msgs::ControllerState::HEIGHT | hector_uav_msgs::ControllerState::HEADING);
      if (controlModeSwitch == 1) switchControlMode(hector_uav_msgs::ControllerState::VELOCITY | hector_uav_msgs::ControllerState::HEIGHT | hector_uav_msgs::ControllerState::HEADING);
      setControlSource(CONTROL_REMOTE);

    } else if (getControlSource() == CONTROL_REMOTE) {
      switchControlMode(hector_uav_msgs::ControllerState::POSITION | hector_uav_msgs::ControllerState::HEIGHT | hector_uav_msgs::ControllerState::HEADING);
      setControlSource(CONTROL_AUTONOMOUS);
    }
  }

  if (rc->getSwitch(SWITCH_HEIGHT_MODE, controlModeSwitch)) {
    if (controlModeSwitch == 0) { altimeter->setMode(Altimeter::Height::OFF); }
    if (controlModeSwitch == 1) { altimeter->clearMode(Altimeter::Height::OFF | Altimeter::Height::AUTO); altimeter->setMode(Altimeter::Height::ULTRASOUND); }
    if (controlModeSwitch == 2) { altimeter->clearMode(Altimeter::Height::OFF); altimeter->setMode(Altimeter::Height::AUTO); }
  }

  // set control mode from autonomy
  if (getControlSource() == CONTROL_AUTONOMOUS) {
    unsigned int newControlMode;
    if (portControlMode.readNewest(newControlMode) == RTT::NewData) {
      setControlMode(newControlMode);
    }
  }

  // switch motors on and off based on vertical velocity command
  if (getControlSource() != CONTROL_AUTONOMOUS) {
    if (controller->height->getRateInput().z > 0.5 && !getState(hector_uav_msgs::ControllerState::AIRBORNE)) {
      engage();
    } else if (controller->height->getRateInput().z < -0.5 && !getState(hector_uav_msgs::ControllerState::AIRBORNE)) {
      shutdown();
    }
  }

  return true;
}

void Quadrotor::afterExecuteHook() {
  controller_state.source = getControlSource();
  controller_state.header.stamp = getTimestamp();
  portControllerState.write(controller_state);

  // reset controllers if motors are not running
  // if (!getState(hector_uav_msgs::ControllerState::MOTORS_RUNNING) && !getState(hector_uav_msgs::ControllerState::AIRBORNE)) reset();
  if (!getState(hector_uav_msgs::ControllerState::MOTORS_RUNNING)) reset();

  // output current velocity commands
  {
    geometry_msgs::Twist command;
    command.linear.x = controller->velocity->getInput().x;
    command.linear.y = controller->velocity->getInput().y;
    command.linear.z = controller->height->getRateInput().z;
    command.angular.z = controller->heading->getRateInput().turnrate;
    portCommandVelocityInput.write(command);
  }
}

void Quadrotor::resetHook() {
  // acceleration = geometry_msgs::Vector3();
  // velocity.x = velocity.y = velocity.z = std::numeric_limits<double>::quiet_NaN();
}

bool Quadrotor::switchControlMode(Mode controlMode, Mode disableMode) {
  Mode newControlMode = getControlMode();
  Mode mask;

  mask = hector_uav_msgs::ControllerState::ATTITUDE | hector_uav_msgs::ControllerState::VELOCITY | hector_uav_msgs::ControllerState::POSITION;
  switch(controlMode & mask) {
    case hector_uav_msgs::ControllerState::ATTITUDE:
      newControlMode &= ~mask;
      newControlMode |= hector_uav_msgs::ControllerState::ATTITUDE;
      break;
    case hector_uav_msgs::ControllerState::VELOCITY:
      newControlMode &= ~mask;
      newControlMode |= hector_uav_msgs::ControllerState::VELOCITY;
      break;
    case hector_uav_msgs::ControllerState::POSITION:
      newControlMode &= ~mask;
      newControlMode |= hector_uav_msgs::ControllerState::POSITION;
      break;
    default:
      break;
  }

  if (controlMode & hector_uav_msgs::ControllerState::HEADING) newControlMode |= hector_uav_msgs::ControllerState::HEADING;
  if (controlMode & hector_uav_msgs::ControllerState::HEIGHT)  newControlMode |= hector_uav_msgs::ControllerState::HEIGHT;
  if (controlMode & hector_uav_msgs::ControllerState::MOTORS)  newControlMode |= hector_uav_msgs::ControllerState::MOTORS;

  newControlMode &= ~disableMode;

  return setControlMode(newControlMode);
}

bool Quadrotor::setControlMode(Mode newControlMode, bool force) {
  Mode currentControlMode = getControlMode();

  if (!force) {
    // we have a request to change to a new control mode, but a previous call failed, do nothing...
    if (newControlMode == requestedControlMode) return false;
    requestedControlMode = newControlMode;
  }

  // check new control mode
//  if (newControlMode & (hector_uav_msgs::ControllerState::VELOCITY | hector_uav_msgs::ControllerState::POSITION) && (nav.insStatus != nav.NAVIGATION_READY)) {
//    RTT::log(RTT::Error) << "Switching to velocity/position control mode failed: Navigation not ready!" << RTT::endlog();
//    return false;
//  }
  if (newControlMode & hector_uav_msgs::ControllerState::POSITION) newControlMode |= hector_uav_msgs::ControllerState::VELOCITY;
  if (newControlMode & hector_uav_msgs::ControllerState::VELOCITY) newControlMode |= hector_uav_msgs::ControllerState::ATTITUDE;

  // switch control mode
  if (newControlMode == currentControlMode && !force) {
    // the new control mode is the current control mode, nothing to be done...
    return true;
  }
  controller_state.mode = newControlMode;

  // configure new control mode
  Mode start = newControlMode & ~currentControlMode;
  Mode stop  = currentControlMode & ~newControlMode;

  if (hector_uav_msgs::ControllerState::ATTITUDE & start) {
    controller->attitude->start();
    RTT::log(RTT::Info) << "Enabling ATTITUDE controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::ATTITUDE & stop) {
    controller->attitude->stop();
    RTT::log(RTT::Warning) << "Disabling ATTITUDE controller!!!!!" << RTT::endlog();
  }

  if (hector_uav_msgs::ControllerState::VELOCITY & start) {
    controller->velocity->start();
    RTT::log(RTT::Info) << "Enabling VELOCITY controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::VELOCITY & stop) {
    controller->velocity->stop();
    RTT::log(RTT::Info) << "Disabling VELOCITY controller" << RTT::endlog();
  }

  if (hector_uav_msgs::ControllerState::POSITION & start) {
    controller->position->start();
    RTT::log(RTT::Info) << "Enabling POSITION controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::POSITION & stop) {
    controller->position->stop();
    RTT::log(RTT::Info) << "Disabling POSITION controller" << RTT::endlog();
  }

  if (hector_uav_msgs::ControllerState::HEADING & start) {
    controller->heading->start();
    RTT::log(RTT::Info) << "Enabling HEADING controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::HEADING & stop) {
    controller->heading->stop();
    RTT::log(RTT::Info) << "Disabling HEADING controller" << RTT::endlog();
  }

  if (hector_uav_msgs::ControllerState::HEIGHT & start) {
    controller->height->start();
    RTT::log(RTT::Info) << "Enabling HEIGHT controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::HEIGHT & stop) {
    controller->height->stop();
    RTT::log(RTT::Info) << "Disabling HEIGHT controller" << RTT::endlog();
  }

  if (hector_uav_msgs::ControllerState::MOTORS & start) {
    controller->motor->start();
    controller->motor->engage();
    RTT::log(RTT::Info) << "Enabling MOTOR controller" << RTT::endlog();
  } else if (hector_uav_msgs::ControllerState::MOTORS & stop) {
    controller->motor->stop();
    RTT::log(RTT::Info) << "Disabling MOTOR controller" << RTT::endlog();
  }

  return true;
}

Mode Quadrotor::getControlMode() const {
  return controller_state.mode;
}

static inline void logState(bool set, State state) {
  if (!state) return;

  std::string message;
  if (set)
    message = "Entering state ";
  else
    message = "Leaving state ";

  if (state & hector_uav_msgs::ControllerState::MOTORS_RUNNING) message += "MOTORS_RUNNING";
  if (state & hector_uav_msgs::ControllerState::AIRBORNE)       message += "AIRBORNE";

  RTT::log(RTT::Info) << message << RTT::endlog();
}

void Quadrotor::setState(State set) {
  logState(true, set & ~controller_state.state);
  controller_state.state |= set;
}

void Quadrotor::clearState(State clear) {
  logState(false, clear & controller_state.state);
  controller_state.state &= ~clear;
}

State Quadrotor::getState() const {
  return controller_state.state;
}

bool Quadrotor::getState(State mask) const {
  return controller_state.state & mask;
}

geometry_msgs::WrenchStamped Quadrotor::getWrench() const {
  return controller->attitude->getOutput() + controller->heading->getOutput() + controller->height->getOutput() + controller->thrust->getOutput();
}

void Quadrotor::writeOutput(const hector_uav_msgs::MotorPWM &output) const {
  controller->motor->getOutputPort()->write(output);
}

void Quadrotor::engage() {
  controller->motor->engage();
}

void Quadrotor::shutdown() {
  controller->motor->shutdown();
}

namespace {
  typedef Quadrotor Controller;
  ORO_CREATE_COMPONENT( Controller )
}

} // namespace Controller
} // namespace uxvcos
