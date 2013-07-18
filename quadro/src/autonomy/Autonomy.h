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

#ifndef AUTONOMY_AUTONOMY_H
#define AUTONOMY_AUTONOMY_H

#include <rtt/TaskContext.hpp>
#include <types/autonomy.h>

namespace Autonomy {

class Autonomy : public RTT::TaskContext
{
public:
  Autonomy(const std::string& name = "Autonomy");
  virtual ~Autonomy();

  bool takeoff();
  bool land();
  bool emergency();
  virtual void reset();

protected:
  virtual bool configureHook();
  virtual void cleanupHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

  virtual void sendCommand(unsigned int command);
  virtual bool inState(unsigned int state);

protected:
  RTT::InputPort<Data::Autonomy::State>  portAutonomyCommand;
  RTT::OutputPort<Data::Autonomy::State> portAutonomyState;
  Data::Autonomy::State current_state;

  RTT::os::Mutex updateMutex;
};

} // namespace Autonomy

#endif // AUTONOMY_AUTONOMY_H
