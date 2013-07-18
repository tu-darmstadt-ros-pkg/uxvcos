//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include "watchdog/watchdog.h"
#include <rtt/Component.hpp>

namespace uxvcos {

ORO_CREATE_COMPONENT(Watchdog)

Watchdog::Watchdog(const std::string& name)
  : RTT::TaskContext(name)
{
  interval = 10.0; // default interval is 10s
  this->addProperty("interval", interval).doc("Interval for periodic activity checks of all peers in seconds");
}

Watchdog::~Watchdog()
{
}

bool Watchdog::startHook()
{
  this->setPeriod(interval);
  return true;
}

void Watchdog::updateHook()
{
  using namespace RTT;
  Logger::In in(getName());

  TaskContext::PeerList peers = this->getPeerList();
  for(TaskContext::PeerList::const_iterator it = peers.begin(); it != peers.end(); ++it) {
    RTT::TaskContext *peer = this->getPeer(*it);

    // recover from exceptions
    if (peer->inException()) {
      log(Warning) << "Peer " << peer->getName() << " is in exception state! Will try to recover..." << endlog();
      if (peer->recover()) {
        log(Info) << "Successfully recovered " << peer->getName() << "." << endlog();
      } else {
        log(Error) << "Error during recovery of peer " << peer->getName() << "." << endlog();
      }
    }

    // recover from error state
    if (peer->inRunTimeError()) {
      log(Warning) << "Peer " << peer->getName() << " is in run-time error state! Will try to recover..." << endlog();
      peer->stop();
      if (peer->start()) {
        log(Info) << "Successfully restarted " << peer->getName() << "." << endlog();
      } else {
        log(Error) << "Error during restart of peer " << peer->getName() << "." << endlog();
      }
    }

    // configure peers
    if (!peer->isConfigured()) {
      log(Warning) << "Peer " << peer->getName() << " is not configured! Reconfiguring ..." << endlog();
      peer->cleanup();
      if (peer->configure()) {
        log(Info) << "Successfully configured " << peer->getName() << "." << endlog();
      } else {
        log(Error) << "Error during configuration of peer " << peer->getName() << "." << endlog();
      }
    }

    // restart peers
    if (!peer->isRunning()) {
      log(Warning) << "Peer " << peer->getName() << " is not running! Restarting ..." << endlog();
      if (peer->start()) {
        log(Info) << "Successfully restarted " << peer->getName() << "." << endlog();
      } else {
        log(Error) << "Error during restart of peer " << peer->getName() << "." << endlog();
      }
    }
  }
}

} // namespace uxvcos
