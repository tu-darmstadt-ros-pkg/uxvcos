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

#include "Attitude.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Attitude::Attitude(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, geometry_msgs::WrenchStamped, hector_uav_msgs::AttitudeCommand, geometry_msgs::Vector3Stamped>(controller, name, description)
    , functionAttitude(this)
    , angular_velocity("Rate")
    , rollpitch("Angle")
    , angular_velocity_x(angular_velocity)
    , roll(rollpitch, true)
    , angular_velocity_y(angular_velocity)
    , pitch(rollpitch, true)
  {
    this->addProperty(functionAttitude);
    this->addProperty(angular_velocity);
    this->addProperty(rollpitch);

    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addPort(portRateOutput);

    controller->addOperation("setRoll", &Attitude::setRoll, this, RTT::OwnThread).doc("Set commanded roll angle").arg("value", "in degrees");
    controller->addOperation("setPitch", &Attitude::setPitch, this, RTT::OwnThread).doc("Set commanded pitch angle").arg("value", "in degrees");

    output.header.frame_id = "/base_link";
  }

  void Attitude::setRoll(double value) {
    input.roll = value * M_PI/180;
    input.header.stamp = controller->getTimestamp();
  }

  void Attitude::setPitch(double value) {
    input.pitch = value * M_PI/180;
    input.header.stamp = controller->getTimestamp();
  }

  bool Attitude::update(double dt) {
    if (controller->getControlSource() == CONTROL_REMOTE && !(controller->getControlMode() & hector_uav_msgs::ControllerState::VELOCITY)) {
      functionAttitude.get(hector_uav_msgs::RC::ROLL,  input.roll);
      functionAttitude.get(hector_uav_msgs::RC::PITCH, input.pitch);
      input.header.stamp = functionAttitude.getTimestamp();
    }

//    roll.set(sin(input.roll));
//    output.wrench.torque.x = roll.update(sin(controller->euler.roll), dt, controller->imu.angular_velocity.x);

//    pitch.set(sin(input.pitch));
//    output.wrench.torque.y = pitch.update(cos(controller->euler.roll) * sin(controller->euler.pitch), dt, controller->imu.angular_velocity.y);

    roll.set(sin(input.roll));
    rate_output.vector.x = roll.update(sin(controller->euler.roll), dt, controller->imu.angular_velocity.x) + roll.dw2;

    angular_velocity_x.set(rate_output.vector.x);
    output.wrench.torque.x = angular_velocity_x.update(controller->imu.angular_velocity.x, dt);

    pitch.set(sin(input.pitch));
    rate_output.vector.y = pitch.update(cos(controller->euler.roll) * sin(controller->euler.pitch), dt, controller->imu.angular_velocity.y) + pitch.dw2;

    angular_velocity_y.set(rate_output.vector.y);
    output.wrench.torque.y = angular_velocity_y.update(controller->imu.angular_velocity.y, dt);

    output.wrench.torque.x *= 2 * l_m * PWM2N;
    output.wrench.torque.y *= 2 * l_m * PWM2N;

    return true;
  }

  void Attitude::reset() {
    roll.reset();
    pitch.reset();
    angular_velocity_x.reset();
    angular_velocity_y.reset();
    rate_output.vector.x = 0.0;
    rate_output.vector.y = 0.0;
    output.wrench.torque.x = 0.0;
    output.wrench.torque.y = 0.0;
  }

} // namespace Controller
} // namespace uxvcos
