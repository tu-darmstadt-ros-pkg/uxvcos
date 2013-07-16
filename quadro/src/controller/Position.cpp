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

#include "Position.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Position::Position(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, hector_uav_msgs::VelocityXYCommand, hector_uav_msgs::PositionXYCommand>(controller, name, description)
    , px(param)
    , py(param)
  {
    this->addProperty(param);
    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addOperation("setPosition", &Position::setPosition, this, RTT::OwnThread).doc("Set commanded position").arg("p_n", "in m").arg("p_e", "in m");
  }

  void Position::setPosition(double x, double y) {
    input.x = x;
    input.y = y;
    input.header.stamp = controller->getTimestamp();
  }

  bool Position::update(double dt) {
    output.header.frame_id = "/nav";

    px.set(input.x);
    output.x = px.update(controller->state.pose.pose.position.x, dt, controller->state.twist.twist.linear.x);

    py.set(input.y);
    output.y = py.update(controller->state.pose.pose.position.y, dt, controller->state.twist.twist.linear.y);

    return true;
  }

  void Position::reset() {
    setPosition(controller->state.pose.pose.position.x, controller->state.pose.pose.position.y);
    px.reset();
    py.reset();
    output.x = 0.0;
    output.y = 0.0;
  }

} // namespace Controller
} // namespace uxvcos
