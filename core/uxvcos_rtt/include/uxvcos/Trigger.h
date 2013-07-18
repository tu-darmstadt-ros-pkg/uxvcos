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

#ifndef UXVCOS_TRIGGER_H
#define UXVCOS_TRIGGER_H

#include <rtt/TaskContext.hpp>
#include <boost/function.hpp>
#include <rtt/extras/SequentialActivity.hpp>

namespace uxvcos {

class Trigger
{
public:
  typedef boost::function<void(RTT::base::PortInterface* port)> TriggerFunction;

  Trigger(const std::string& name = std::string()) : tc(name, RTT::TaskContext::Running) {
    tc.setActivity(new RTT::extras::SequentialActivity);
  }

  Trigger(const TriggerFunction& func, const std::string& name = std::string()) : tc(name, RTT::TaskContext::Running) {
    *this = func;
    tc.setActivity(new RTT::extras::SequentialActivity);
  }

  virtual ~Trigger() {}

  Trigger& operator=(const TriggerFunction& func) {
    this->func = func;
    return *this;
  }

  RTT::base::InputPortInterface& addPort(RTT::base::InputPortInterface& port) {
    return tc.ports()->addLocalEventPort(port, boost::bind(&Trigger::dataOnPort, this, _1));
  }

protected:
  void dataOnPort(RTT::base::PortInterface* port) {
    if (!func) return;
    func(port);
  }

private:
  RTT::TaskContext tc;
  TriggerFunction func;
};

} // namespace uxvcos

#endif // UXVCOS_TRIGGER_H
