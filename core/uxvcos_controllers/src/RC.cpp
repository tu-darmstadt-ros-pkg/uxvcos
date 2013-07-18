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

#include "RC.h"
#include "ControllerTask.h"
#include "Controller.h"

#include <hector_uav_msgs/RC/functions.h>

namespace uxvcos {
namespace Controller {

RC::RC(ControllerTask *owner)
  : Module(owner, "RC", "Input from remote control")
  , portRC("rc")
  , timeout("Timeout", "RC signal is considered as invalid after this time", 1.0)
  , owner(owner)
{
  this->addPort(portRC);
  owner->addPort(portRC);
}

RC::~RC()
{}

void RC::execute()
{
  read();
}

bool RC::read()
{
  if (portRC.read(rc) == RTT::NoData ||
      (owner->getTimestamp() - rc.header.stamp).toSec() > timeout) {
    rc.valid = false;
    return false;
  }
  return true;
}

bool RC::valid() const {
  return rc.valid;
}

bool RC::hasAxis(RC::AxisFunctionType function) const {
  return hector_uav_msgs::hasAxis(rc, function);
}

bool RC::getAxis(RC::AxisFunctionType function, AxisType &value) const {
  return hector_uav_msgs::getAxis(rc, function, value);
}

bool RC::hasSwitch(RC::SwitchFunctionType function) const {
  return hector_uav_msgs::hasSwitch(rc, function);
}

bool RC::getSwitch(RC::SwitchFunctionType function, SwitchType &value) const {
  return hector_uav_msgs::getSwitch(rc, function, value);
}

uxvcos::Time RC::getTimestamp() const {
  return rc.header.stamp;
}

RC::Function::Function(BaseController *controller, const std::string& name, const std::string& description)
  : Child(name.empty() ? std::string("Input") : name, description)
  , transformation("Transformation")
  , deadzone("Deadzone", "Dead zone (in fraction of 1.0) where action is set to zero")
  , failsafe("Failsafe", "Default value in case of RC failure")
  , controller(controller)
{
  this->declareProperty(transformation);
  this->declareProperty(deadzone);
  this->declareProperty(failsafe);
}

double RC::Function::operator()(double value) const {
  if (value > 0.0) {
    if (value < deadzone)
      value = 0.0;
    else
      value = (value - deadzone) / (1.0 - deadzone);
  } else if (value < 0.0) {
    if (value > -deadzone)
      value = 0.0;
    else
      value = (value + deadzone) / (1.0 - deadzone);
  }

  return transformation(value);
}

bool RC::Function::get(RC::AxisFunctionType function, float &value) const {
  RC::AxisType input;
  if (!controller->getOwner()->getRC() ||
      !controller->getOwner()->getRC()->getAxis(function, input)) {
    if (failsafe.ready()) value = failsafe.get();
    return false;
  }
  value = operator()(input);
  return true;
}

uxvcos::Time RC::Function::getTimestamp() const {
  if (!controller->getOwner()->getRC()) return uxvcos::Time();
  return controller->getOwner()->getRC()->getTimestamp();
}


} // namespace Controller
} // namespace uxvcos
