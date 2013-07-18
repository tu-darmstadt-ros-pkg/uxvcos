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

#include "Autonomy.h"
#include <rtt/os/Mutex.hpp>

namespace Autonomy {

Autonomy::Autonomy(const std::string& name)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  , portAutonomyCommand("AutonomyCommand")
  , portAutonomyState("AutonomyState")
{
  this->addOperation("takeoff", &Autonomy::takeoff, this, RTT::ClientThread);
  this->addOperation("land", &Autonomy::land, this, RTT::ClientThread);
  this->addOperation("emergency", &Autonomy::emergency, this, RTT::ClientThread);
  this->addOperation("reset", &Autonomy::reset, this, RTT::ClientThread);
}

Autonomy::~Autonomy()
{}

bool Autonomy::takeoff() {
  sendCommand(Data::Autonomy::TAKEOFF);
  return inState(Data::Autonomy::TAKEOFF);
}

bool Autonomy::land() {
  sendCommand(Data::Autonomy::LANDING);
  return inState(Data::Autonomy::LANDING);
}

bool Autonomy::emergency() {
  sendCommand(Data::Autonomy::EMERGENCY);
  return inState(Data::Autonomy::EMERGENCY);
}

void Autonomy::reset() {
  sendCommand(Data::Autonomy::NONE);
}

void Autonomy::sendCommand(unsigned int command) {
}

bool Autonomy::inState(unsigned int state) {
  updateMutex.trylock();
  if (!updateMutex.timedlock(1.0)) {
    RTT::log(RTT::Error) << "Could not retrieve state information. Is Autonomy currently running?" << RTT::endlog();
    return false;
  }
  bool result = (current_state.state & state) > 0;
  updateMutex.unlock();

  RTT::log(RTT::Info) << "Autonomy is in state " << current_state.state << " now." << RTT::endlog();
  return result;
}

bool Autonomy::configureHook() {
  return true;
}

void Autonomy::cleanupHook() {
}

bool Autonomy::startHook() {
  return true;
}

void Autonomy::updateHook() {
}

void Autonomy::stopHook() {
}

} // namespace Autonomy

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( Autonomy::Autonomy )
