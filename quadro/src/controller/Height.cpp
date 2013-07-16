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

#include "Height.h"
#include "Altimeter.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Height::Height(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, geometry_msgs::WrenchStamped, hector_uav_msgs::HeightCommand, hector_uav_msgs::VelocityZCommand>(controller, name, description)
    , functionClimbrate(this)
    , pid_height(false, "Height")
    , pid_climbrate(false, "Rate")
    , maximumLoadFactor("MaximumLoadFactor", "Maximum load factor considered for height control", 1.2)
    , loadFactor(0.0)
    , oldMode(0)
  {
    this->addProperty(functionClimbrate);
    this->addProperty(pid_height.parameters());
    this->addProperty(pid_climbrate.parameters());

    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addPort(portRateCommand);
    controller->addPort(portRateInput);
    controller->addPort(portRateOutput);

    controller->addOperation("setHeight", &Height::setHeight, this, RTT::OwnThread).doc("Set height manually.").arg("value", "new value in m");
    controller->addOperation("setClimbrate", &Height::setClimbrate, this, RTT::OwnThread).doc("Set climbrate manually.").arg("value", "new value in m/s");

    this->addAttribute("loadFactor", loadFactor);
    this->addProperty(maximumLoadFactor);

    output.header.frame_id = "/base_link";
  }

  bool Height::update(RTT::FlowStatus input_status, double dt) {
    // check if control mode has changed and step input height
    if (oldMode != controller->altimeter->getMode()) {
      if (controller->altimeter->getMode() & Altimeter::Height::OFF) {
        if (!(oldMode & Altimeter::Height::OFF)) RTT::log(RTT::Info) << "Height controller switched off!" << RTT::endlog();

      } else {
        if (oldMode & Altimeter::Height::OFF) {
          RTT::log(RTT::Info) << "Height controller switched on!" << RTT::endlog();

          // set the input.height to the current height
          input.height = pid_height.jump(controller->altimeter->getHeight());
          input.header.stamp = controller->getTimestamp();
          RTT::log(RTT::Info) << "Jumped to new input height equal to current height " << input.height << " m" << RTT::endlog();

        } else if (oldMode != 0 && !isnan(pid_height.y)) {
          // step the input.height to prevent a jump in the control error
          input.height = pid_height.jump(input.height - pid_height.y + controller->altimeter->getHeight());
          input.header.stamp = controller->getTimestamp();
          RTT::log(RTT::Info) << "Jumped to new " << controller->altimeter->getModeString() << "input height " << input.height << " m" << RTT::endlog();
        }
      }

      oldMode = controller->altimeter->getMode();
    }

    // RC input
    if (controller->getControlSource() == CONTROL_REMOTE) {
      float vz;
      if (functionClimbrate.get(hector_uav_msgs::RC::HEIGHT, vz)) {
        rate_input.z = vz;
        rate_input.header.stamp = functionClimbrate.getTimestamp();
      }
    }

    // stop climb/descent if a new height has been received
    if (input_status == RTT::NewData) {
      rate_input.z = 0.0;
      rate_input.header.stamp = controller->getTimestamp();
    }

//    // only enable controller if AIRBORNE (otherwise return last commanded force)
//    if (!controller->getState(hector_uav_msgs::ControllerState::AIRBORNE)) {
//      return true;
//    }

    // wait for auto throttle before height controller is activated
    if (!controller->getState(hector_uav_msgs::ControllerState::MOTORS_RUNNING)) {
      wait_for_autothrottle = true;
      return true;
    } else if (controller->getState(hector_uav_msgs::ControllerState::AIRBORNE)) {
      // auto throttle finished, height controller is active
      wait_for_autothrottle = false;
    } else if (wait_for_autothrottle) {
      // auto throttle could be active, do nothing (just return the last commanded thrust)
      return true;
    }

    // read commanded vertical speed and height
    if (rate_input.z != 0.0 && !(controller->altimeter->getMode() & Altimeter::Height::OFF)) {
      input.height += dt * rate_input.z;
      input.header.stamp = controller->getTimestamp();
    }

    // calculate PID output
    if (!(controller->altimeter->getMode() & Altimeter::Height::OFF)) {
      pid_height.set(input.height);
      rate_output.z = rate_input.z + pid_height.update(controller->altimeter->get().height, dt, controller->altimeter->get().vertical_speed);

      pid_climbrate.set(rate_output.z);
      output.wrench.force.z = pid_climbrate.update(controller->altimeter->get().vertical_speed, dt);

    } else {
      rate_output.z = rate_input.z;
      output.wrench.force.z = pid_climbrate.parameters().KI * pid_climbrate.e_I + pid_climbrate.parameters().KP * rate_input.z;
    }

    // apply load_factor
    loadFactor = std::min(1.0/cos(controller->euler.roll)/cos(controller->euler.pitch), maximumLoadFactor.get());
    output.wrench.force.z *= loadFactor;

    output.wrench.force.z *= 4 * PWM2N;

    return true;
  }

  void Height::setHeight(double value) {
    input.height = value;
    input.header.stamp = controller->getTimestamp();
  }

  void Height::setClimbrate(double value) {
    rate_input.z = value;
    rate_input.header.stamp = controller->getTimestamp();
  }

  bool Height::beforeStart() {
    setClimbrate(0.0);
    return true;
  }

  void Height::reset() {
    setHeight(pid_height.reset(controller->altimeter->get().height));
    pid_climbrate.reset();
    oldMode = controller->altimeter->getMode();
    rate_output.z = 0.0;
    output.wrench.force.z = 0.0;
    wait_for_autothrottle = true;
  }

} // namespace Controller
} // namespace uxvcos
