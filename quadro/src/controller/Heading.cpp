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

#include "Heading.h"
#include "Quadrotor.h"
#include <uxvcos/controller/helper.h>

namespace uxvcos {
namespace Controller {

  Heading::Heading(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, geometry_msgs::WrenchStamped, hector_uav_msgs::HeadingCommand, hector_uav_msgs::YawrateCommand>(controller, name, description)
    , functionTurnrate(this)
    , pid_heading(true, "Angle")
    , pid_turnrate(false, "Rate")
  {
    this->addProperty(functionTurnrate);
    this->addProperty(pid_heading.parameters());
    this->addProperty(pid_turnrate.parameters());

    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addPort(portRateCommand);
    controller->addPort(portRateInput);
    controller->addPort(portRateOutput);

    controller->addOperation("setHeading", &Heading::setHeading, this, RTT::OwnThread).doc("Set heading manually").arg("value", "in degrees");
    controller->addOperation("setTurnrate", &Heading::setTurnrate, this, RTT::OwnThread).doc("Set turnrate manually").arg("value", "in degrees");

    output.header.frame_id = "/base_link";
  }

  void Heading::setHeading(double value) {
    input.heading = value;
    input.header.stamp = (controller->getTimestamp());
  }

  void Heading::setTurnrate(double value) {
    rate_input.turnrate = value;
    rate_input.header.stamp = (controller->getTimestamp());
  }

  bool Heading::update(RTT::FlowStatus input_status, double dt) {
    // RC input
    float temp;
    if (functionTurnrate.get(hector_uav_msgs::RC::YAW, temp) && (controller->getControlSource() == CONTROL_REMOTE || temp != 0.0)) {
      rate_input.turnrate = temp;
      rate_input.header.stamp = functionTurnrate.getTimestamp();
    }

    // stop turning if a new heading has been received directly
    if (input_status == RTT::NewData) {
      rate_input.turnrate = 0.0;
      rate_input.header.stamp = controller->getTimestamp();
    }

    if (rate_input.turnrate != 0.0) {
      input.heading = helper::modulo(input.heading + dt * rate_input.turnrate, -M_PI, M_PI);
      input.header.stamp = controller->getTimestamp();
    }

    pid_heading.set(input.heading);
    rate_output.turnrate = rate_input.turnrate + pid_heading.update(controller->euler.yaw, dt, controller->imu.angular_velocity.z);

    pid_turnrate.set(rate_output.turnrate);
    output.wrench.torque.z = pid_turnrate.update(controller->imu.angular_velocity.z, dt);
    output.wrench.torque.z *= 4 * PWM2NM;

    return true;
  }

  void Heading::reset() {
    input.heading = controller->euler.yaw;
    pid_heading.reset();
    pid_turnrate.reset();
    rate_output.turnrate = 0.0;
    output.wrench.torque.z = 0.0;
  }

} // namespace Controller
} // namespace uxvcos
