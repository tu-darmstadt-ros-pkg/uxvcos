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

#ifndef CONTROLLER_AIRPLANE_H
#define CONTROLLER_AIRPLANE_H

#include <uxvcos/controller/Controller.h>
#include <uxvcos/controller/PID.h>
#include <uxvcos/controller/RC.h>
#include <uxvcos/controller/ControllerTask.h>

#include <uxvcos/Transformation.h>

#include <hector_uav_msgs/typekit/ServoCommand.h>
#include <hector_uav_msgs/typekit/RuddersCommand.h>
#include <hector_std_msgs/typekit/Float32.h>
#include <hector_uav_msgs/typekit/MotorPWM.h>

namespace uxvcos {
namespace Controller {

class Airplane : public ControllerTask {
public:
  Airplane(const std::string &name = "Controller");
  virtual ~Airplane();

protected:
  virtual bool init();
  virtual bool beforeExecute();
  virtual void afterExecute();
  virtual void reset();
  virtual void stopped();

protected:
  RC rc;

public:
  class Throttle : public Controller<Airplane, hector_uav_msgs::MotorPWM, hector_std_msgs::Float32> {
  public:
    Throttle(Airplane* controller, const std::string& name = "Throttle", const std::string& description = "Throttle Controller");
    bool update(double dt);
    void reset();

  protected:
    RTT::Attribute<bool> on;
  } throttle;

  class Rudder : public Child {
  public:
    Rudder(Child* parent, const std::string& name, const std::string& description = "")
      : Child(name, description)
      , channel("Channel", "Channel number for this axis", 0)
      , transformation(name, "Transformation from rudder angle [-1,1] to servo output")
    {
      properties()->add(&channel);
      properties()->addProperty(transformation);
      transformation.limit = true;
      transformation.minimum = -1.0;
      transformation.maximum = 1.0;

      parent->properties()->addProperty(*this);
    }
    
  public:
    RTT::Property<int> channel;
    LinearTransformation<double,double> transformation;
  };
  
  class Rudders : public Controller<Airplane, hector_uav_msgs::ServoCommand, hector_uav_msgs::RuddersCommand> {
  public:
    Rudders(Airplane* controller, const std::string& name = "Rudders", const std::string& description = "Rudders Controller")
      : Controller<Airplane, hector_uav_msgs::ServoCommand, hector_uav_msgs::RuddersCommand>(controller, name, description)
      , leftaileron(this, "LeftAileron")
      , rightaileron(this, "RightAileron")
      , elevator(this, "Elevator")
      , rudder(this, "Rudder")
      , throttle(this, "Throttle")
    {
    }

    bool update(double dt);
    void reset();

  protected:
    Rudder leftaileron;
    Rudder rightaileron;
    Rudder elevator;
    Rudder rudder;
    Rudder throttle;

  public:
    hector_uav_msgs::RuddersCommand rudders;
    hector_uav_msgs::ServoCommand servos;
  } rudders;
};

} // namespace Controller
} // namespace uxvcos

#endif // CONTROLLER_AIRPLANE_H
