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

#include "ControllerTask.h"
#include "Controller.h"

#include <boost/algorithm/string.hpp>

namespace uxvcos {
namespace Controller {

ControllerTask::ControllerTask(const std::string &name)
  : RTT::TaskContext(name, PreOperational)
  , ModuleContainer(this)
//  , portClock("Clock")
  , _controlSource(0)
  , _dt(0.0)
  , _latency(0.0)
{
  // this->addPort( portClock ).doc("Current Timestamp");
  this->addOperation("reset", &ControllerTask::reset, this).doc("Resets all controllers.");
  this->addConstant("timestamp", _timestamp);
  this->addConstant("timestep", _dt);
  this->addAttribute("latency", _latency);
}

ControllerTask::~ControllerTask() {
}

bool ControllerTask::configureHook() {
  RTT::Logger::In in(getName());
  return initializeModules();
}

bool ControllerTask::startHook() {
  RTT::Logger::In in(getName());
  if (!beforeStart()) return false;

  bool result = true;
  for(Controllers::const_iterator it = controllers().begin(); it != controllers().end(); ++it) {
    ControllerPtr controller = *it;
    if (controller->autostart()) result &= controller->start();
  }

  return result;
}

void ControllerTask::updateHook() {
  RTT::Logger::In in(getName());
//  rosgraph_msgs::Clock clock;
//  if (portClock.read(clock, false) == RTT::NewData) {
//    setTimestamp(clock.clock);
//  }

  if (!beforeExecuteHook()) return;
  if (_latency > 0) usleep(_latency * 1e6);
  executeModules();
  afterExecuteHook();
}

const uxvcos::Time& ControllerTask::setTimestamp(const uxvcos::Time& new_timestamp) {
  _dt = (new_timestamp - _timestamp).toSec();
  if (_dt > 1.0) _dt = 1.0;
  _timestamp = new_timestamp;
  return _timestamp;
}

void ControllerTask::stopHook() {
  RTT::Logger::In in(getName());

  for(Controllers::const_reverse_iterator it = controllers().rbegin(); it != controllers().rend(); ++it) {
    ControllerPtr controller = *it;
    controller->stop();
  }

  afterStop();
}

void ControllerTask::cleanupHook() {
  RTT::Logger::In in(getName());
  cleanupModules();
}

void ControllerTask::reset() {
  RTT::Logger::In in(getName());
  for(Controllers::const_iterator it = controllers().begin(); it != controllers().end(); ++it) {
    (*it)->reset();
  }
  resetHook();
}

ControllerTask::ControllerPtr ControllerTask::getController(const std::string& name) const {
  for(Controllers::const_iterator it = controllers().begin(); it != controllers().end(); ++it) {
    if ((*it)->getName() != name) continue;
    return *it;
  }

  return BaseController::shared_ptr();
}

BaseController::BaseController(ControllerTask* owner, const std::string& name, const std::string& description)
  : Module(owner, name, description)
  , topic(boost::algorithm::to_lower_copy(name))
  , _is_active(false)
  , _autostart(false)
{
  this->addOperation("start", &BaseController::start, this).doc("Starts the " + name + " controller");
  this->addOperation("stop", &BaseController::stop,  this).doc("Stops the " + name + " controller");
  this->addOperation("isActive", &BaseController::isActive, this).doc("Returns true, if the " + name + " controller is active");
}

ControllerTask *BaseController::getOwner() const {
  return static_cast<ControllerTask *>(Module::getOwner());
}

bool BaseController::start() {
  if (!beforeStart()) return false;
  _is_active = true;
  return true;
}

void BaseController::stop() {
  _is_active = false;
  afterStop();
}


} // namespace Controller
} // namespace uxvcos
