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

#ifndef UXVCOS_CONTROLLER_RC_H
#define UXVCOS_CONTROLLER_RC_H

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>

#include <uxvcos/Module.h>
#include <uxvcos/Transformation.h>

#include <uxvcos/Time.h>
#include <hector_uav_msgs/typekit/RC.h>

namespace uxvcos {
namespace Controller {

class ControllerTask;
class BaseController;

class RC : public Module {
public:
  typedef boost::shared_ptr<RC> Ptr;

  typedef hector_uav_msgs::RC::_axis_function_type::value_type AxisFunctionType;
  typedef hector_uav_msgs::RC::_swit_function_type::value_type SwitchFunctionType;
  typedef hector_uav_msgs::RC::_axis_type::value_type AxisType;
  typedef hector_uav_msgs::RC::_swit_type::value_type SwitchType;

  RC(ControllerTask *owner);
  virtual ~RC();

  virtual bool read();
  virtual bool valid() const;

  virtual bool hasAxis(AxisFunctionType function) const;
  virtual bool getAxis(AxisFunctionType function, AxisType &value) const;
  virtual bool hasSwitch(SwitchFunctionType function) const;
  virtual bool getSwitch(SwitchFunctionType function, SwitchType &value) const;

  virtual uxvcos::Time getTimestamp() const;

protected:
  virtual void execute();

protected:
  RTT::InputPort<hector_uav_msgs::RC> portRC;
  hector_uav_msgs::RC rc;
  RTT::Property<double> timeout;
  ControllerTask *owner;

public:
  class Function : public Child {
  public:
    Function(BaseController *controller, const std::string& name = "", const std::string& description = "");

    double operator()(double value) const;
    bool get(AxisFunctionType function, float &value) const;
    uxvcos::Time getTimestamp() const;

  public:
    LinearTransformation<double,double> transformation;
    RTT::Property<double> deadzone;
    RTT::Property<double> failsafe;

  private:
    BaseController *controller;
  };

}; // class RC

} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_FUNCTION_H
