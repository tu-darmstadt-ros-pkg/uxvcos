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

#ifndef INTERFACE_SERVOS_H
#define INTERFACE_SERVOS_H

#include <uxvcos/Module.h>
#include <uxvcos/Trigger.h>

#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

#include <hector_uav_msgs/typekit/ServoCommand.h>

namespace uxvcos {

class Servos : public Module {
public:
  class Handler {
    public:
      virtual ~Handler() {}
      virtual bool newServoCommand(const hector_uav_msgs::ServoCommand& servoCommand) = 0;
  };

  Servos(RTT::TaskContext* parent, const std::string& name = "Servos", const std::string& description = "Interface to the Servos")
    : Module(parent, name, description)
    , command("servo_output")
    , handler(dynamic_cast<Handler*>(parent))
  {
    RTT::Logger::In in("Servo");

    command.doc("Servo outputs");
    this->addPort(command);
    parent->addPort(command);

    if (!handler) RTT::log( RTT::Error ) << "No Servo handler defined in " << __FUNCTION__ << RTT::endlog();

    trigger = boost::bind(&Servos::newServoCommand, this, _1);
    trigger.addPort(command);
  }

  virtual ~Servos() {
    parent->removePort(command.getName());
  }

  void newServoCommand(RTT::base::PortInterface* port) {
    if (!handler || port != &command) return;

    hector_uav_msgs::ServoCommand data;
    if (RTT::NewData == command.read(data))
      handler->newServoCommand(data);
  }

public:
  RTT::InputPort <hector_uav_msgs::ServoCommand> command;
  Trigger trigger;

protected:
  Handler *handler;
};

} // namespace uxvcos

#endif // INTERFACE_SERVOS_H
