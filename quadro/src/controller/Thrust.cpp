//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include "Thrust.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Thrust::Thrust(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, geometry_msgs::WrenchStamped, hector_uav_msgs::ThrustCommand>(controller, name, description)
    , functionThrust(this)
    , autoThrust("AutoThrottle", "Throttle should be managed automatically during ", true)
  {
    this->addProperty(functionThrust);
    this->addProperty(autoThrust);
    this->addAttribute("MaximumThrust", maxThrust);

    controller->addPort(portCommand);
    controller->addPort(portInput);

    controller->addOperation("setThrottle", &Thrust::setThrottle, this, RTT::OwnThread).doc("Set throttle manually.").arg("value", "new value (in [0,1])");

    output.header.frame_id = "/base_link";
    maxThrust = 1.0;
    autoThrustState = OFF;
  }

  bool Thrust::update(RTT::FlowStatus status, double dt) {
    // RC input
    float rc_thrust;
    if (functionThrust.get(hector_uav_msgs::RC::THRUST, rc_thrust)) {
      if (rc_thrust < 0.05) {
        controller->shutdown();
        reset();
      }

      if (autoThrust.get() == true) {
        maxThrust = rc_thrust;
      } else if (controller->getControlSource() == CONTROL_REMOTE) {
        input.thrust = rc_thrust;
        input.header.stamp = functionThrust.getTimestamp();
      }
    }

    // disable autoThrust when a throttle command has been received
    if (status == RTT::NewData) autoThrustState = AIRBORNE;

    // set automatic thrust based on the autoThrust property
    if (autoThrust.get() == true) {
      switch(autoThrustState) {
        case OFF:
          if (!controller->getState(hector_uav_msgs::ControllerState::MOTORS_RUNNING) && controller->altimeter->get().height_above_ground > 0.0 && controller->altimeter->get().height_above_ground < 0.30) {
            RTT::log(RTT::Info) << "Auto thrust is ready." << RTT::endlog();
            autoThrustState = IDLE;
          }
          break;

        case IDLE:
          if (!controller->getState(hector_uav_msgs::ControllerState::AIRBORNE) && controller->getState(hector_uav_msgs::ControllerState::MOTORS_RUNNING)) {
            if (!(controller->altimeter->get().height_above_ground > 0.0)) {
              RTT::log(RTT::Warning) << "Auto thrust cannot be initiated when no height sensor is available!" << RTT::endlog();
              autoThrustState = OFF;
              break;
            }

            RTT::log(RTT::Info) << "Initiating auto thrust takeoff sequence" << RTT::endlog();
            autoThrustState = STARTUP;
            break;
          }

          input.thrust = 0.0;
          break;

        case STARTUP:
          if (controller->getState(hector_uav_msgs::ControllerState::AIRBORNE)) {
            RTT::log(RTT::Info) << "Auto thrust takeoff sequence finished with thrust " << output.wrench.force.z << " N" << RTT::endlog();
            autoThrustState = AIRBORNE;
            break;
          }

          if (input.thrust < 0.2) input.thrust += dt * 0.1;
          if (input.thrust < 0.5) input.thrust += dt * 0.2;
          if (input.thrust < 1.0) input.thrust += dt * 0.1;
          if (input.thrust > maxThrust) input.thrust = maxThrust;
          input.header.stamp = controller->getTimestamp();
          break;

        case AIRBORNE:
          if (input.thrust > maxThrust) {
            input.thrust = maxThrust;
            input.header.stamp = controller->getTimestamp();
          }
          break;
      }
    }

    output.wrench.force.z = input.thrust * 255.0;
    output.wrench.force.z *= 4 * PWM2N;

    return true;
  }

  void Thrust::setThrottle(double value) {
    input.thrust = value;
    input.header.stamp = controller->getTimestamp();
    if (value == 0.0)
      autoThrustState = IDLE;
    else
      autoThrustState = AIRBORNE;
  }

  void Thrust::setThrust(double value) {
    setThrottle(value / 255.0 / 4 / PWM2N);
  }

  void Thrust::setOperatingPoint(double value) {
    setThrottle(value / 255.0);
  }

  void Thrust::reset() {
    autoThrustState = OFF;
    if (autoThrust.get() == true) setThrottle(0.0);
    output.wrench.force.z = 0.0;
  }

} // namespace Controller
} // namespace uxvcos
