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

#ifndef QUADRO_CONTROLLER_HEIGHT_H
#define QUADRO_CONTROLLER_HEIGHT_H

#include <controller/Controller.h>
#include <controller/RC.h>
#include <controller/PID.h>
#include <controller/Altimeter.h>

#include <geometry_msgs/typekit/WrenchStamped.h>
#include <hector_uav_msgs/typekit/HeightCommand.h>
#include <hector_uav_msgs/typekit/VelocityZCommand.h>

namespace uxvcos {
namespace Controller {

  class Quadrotor;

  class Height : public Controller<Quadrotor, geometry_msgs::WrenchStamped, hector_uav_msgs::HeightCommand, hector_uav_msgs::VelocityZCommand> {
  public:
    Height(Quadrotor* controller, const std::string& name = "Height", const std::string& description = "Height Controller");
    bool beforeStart();
    bool update(RTT::FlowStatus, double dt);
    void reset();

    void setHeight(double value);
    void setClimbrate(double value);

  private:
    RC::Function functionClimbrate;
    PID pid_height, pid_climbrate;
    RTT::Property<double> maximumLoadFactor;
    double loadFactor;
    Altimeter::Mode oldMode;

    bool wait_for_autothrottle;
  };

} // namespace Controller
} // namespace uxvcos

#endif // QUADRO_CONTROLLER_HEIGHT_H
