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

#include "Airplane.h"

#include <rtt/Logger.hpp>
#include <uxvcos/controller/Limit.h>

namespace uxvcos {
namespace Controller {

Airplane::Airplane(const std::string &name)
  : ControllerTask(name)
  , rc(this)
  , throttle(this)
  , rudders(this)
{
  this->addModule(&throttle);
  this->addModule(&rudders);
}

Airplane::~Airplane() {
  stop();
  cleanup();
}

void Airplane::reset() {
  throttle.reset();
  rudders.reset();
}

Airplane::Throttle::Throttle(Airplane* controller, const std::string& name, const std::string& description)
  : Controller<Airplane, hector_uav_msgs::MotorPWM, hector_std_msgs::Float32>(controller, name, description)
  , on("on")
{
  controller->provides()->addAttribute(on);
}

bool Airplane::Throttle::update(double dt) {
  const Limit<double> limit(10,120);

  reset();
  if (on.get()) {
    output.pwm[0] = static_cast<unsigned char>(limit(120 * input.data));
  }

  return true;
}

void Airplane::Throttle::reset() {
  output.pwm[0] = 0;
  output.pwm[1] = 0;
  output.pwm[2] = 0;
  output.pwm[3] = 0;
}

bool Airplane::Rudders::update(double dt) {
  for(std::size_t i = 0; i < output.value.size(); i++) output.value[i] = 0;
  
  if (leftaileron.channel > 0 && leftaileron.channel <= output.value.size())
    output.value[leftaileron.channel-1]  = static_cast<uint16_t>(1500.0 + 400.0 * leftaileron.transformation(input.aileron));

  if (rightaileron.channel > 0 && rightaileron.channel <= output.value.size())
    output.value[rightaileron.channel-1] = static_cast<uint16_t>(1500.0 + 400.0 * rightaileron.transformation(input.aileron));

  if (elevator.channel > 0 && elevator.channel <= output.value.size())
    output.value[elevator.channel-1]     = static_cast<uint16_t>(1500.0 + 400.0 * elevator.transformation(input.elevator));

  if (rudder.channel > 0 && rudder.channel <= output.value.size())
    output.value[rudder.channel-1]       = static_cast<uint16_t>(1500.0 + 400.0 * rudder.transformation(input.rudder));

  if (throttle.channel > 0 && throttle.channel <= output.value.size())
    output.value[throttle.channel-1]     = static_cast<uint16_t>(1500.0 + 400.0 * (2 * throttle.transformation(getOwner()->throttle.getOutput().pwm[0]) - 1));

  return true;
}

void Airplane::Rudders::reset() {
}

} // namespace Controller
} // namespace uxvcos
