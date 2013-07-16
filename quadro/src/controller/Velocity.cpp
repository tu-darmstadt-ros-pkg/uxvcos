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

#include "Velocity.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Velocity::Velocity(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, hector_uav_msgs::AttitudeCommand, hector_uav_msgs::VelocityXYCommand>(controller, name, description)
    , functionVelocity(this)
    , vx(param)
    , vy(param)
  {
    this->addProperty(functionVelocity);
    this->addProperty(param);

    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addOperation("setVelocity", &Velocity::setVelocity, this, RTT::OwnThread).doc("Set commanded velocity").arg("v_x", "in m/s").arg("v_y", "in m/s");

    output.header.frame_id = "/base_link";
  }

  void Velocity::setVelocity(double vx, double vy) {
    input.x = vx;
    input.y = vy;
    input.header.stamp = controller->getTimestamp();
    input.header.frame_id = "/base_link";
  }

  bool Velocity::update(double dt) {
    if (controller->getControlSource() == CONTROL_REMOTE && (controller->getControlMode() & hector_uav_msgs::ControllerState::ATTITUDE)) {
      functionVelocity.get(hector_uav_msgs::RC::PITCH, input.x);
      functionVelocity.get(hector_uav_msgs::RC::ROLL, input.y); input.y = -input.y;
      input.header.stamp = functionVelocity.getTimestamp();
      input.header.frame_id = "/base_link";
    }

    double nav_vx = 0.0, nav_vy = 0.0;

    const double sin_yaw = sin(controller->euler.yaw);
    const double cos_yaw = cos(controller->euler.yaw);

    if (input.header.frame_id == "/nav") {
      nav_vx = input.x;
      nav_vy = input.y;
    } else if (input.header.frame_id == "/base_link") {
      nav_vx = cos_yaw * input.x - sin_yaw * input.y;
      nav_vy = sin_yaw * input.x + cos_yaw * input.y;
    }

    // calculate contol outputs
    vx.set(nav_vx);
    double attitude_x = vx.update(controller->state.twist.twist.linear.x, dt);
    vy.set(nav_vy);
    double attitude_y = vy.update(controller->state.twist.twist.linear.y, dt);

    // NED
    // output.pitch = -cos_yaw * attitude_x - sin_yaw * attitude_y;
    // output.roll  = -sin_yaw * attitude_x + cos_yaw * attitude_y;

    // NWU
    output.pitch =   cos_yaw * attitude_x + sin_yaw * attitude_y;
    output.roll  =   sin_yaw * attitude_x - cos_yaw * attitude_y;

    return true;
  }

  void Velocity::reset() {
    vx.reset();
    vy.reset();
    output.pitch = 0.0;
    output.roll  = 0.0;
  }

} // namespace Controller
} // namespace uxvcos
