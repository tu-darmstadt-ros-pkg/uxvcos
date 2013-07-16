//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#ifndef QUADRO_CONTROLLER_VELOCITY_H
#define QUADRO_CONTROLLER_VELOCITY_H

#include <controller/Controller.h>
#include <controller/RC.h>
#include <controller/PID.h>

#include <hector_uav_msgs/typekit/AttitudeCommand.h>
#include <hector_uav_msgs/typekit/VelocityXYCommand.h>
#include <geometry_msgs/Vector3.h>

namespace uxvcos {
namespace Controller {

  class Quadrotor;

  class Velocity : public Controller<Quadrotor, hector_uav_msgs::AttitudeCommand, hector_uav_msgs::VelocityXYCommand> {
  public:
    Velocity(Quadrotor* controller, const std::string& name = "Velocity", const std::string& description = "Velocity Controller");
    bool update(double dt);
    void reset();

    void setVelocity(double vx, double vy = 0.0);

  private:
    RC::Function functionVelocity;
    PID::Parameter param;
    PID vx, vy;

    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 acceleration;
  };

} // namespace Controller
} // namespace uxvcos

#endif // QUADRO_CONTROLLER_VELOCITY_H
