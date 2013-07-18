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

#ifndef UXVCOS_CONTROLLER_CONTROLLERTASK_H
#define UXVCOS_CONTROLLER_CONTROLLERTASK_H

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <uxvcos/ModuleContainer.h>

#include <uxvcos/Time.h>
#include <hector_uav_msgs/ControlSource.h>

namespace uxvcos {
namespace Controller {

class BaseController;
class RC;
using hector_uav_msgs::ControlSource;
using hector_uav_msgs::CONTROL_AUTONOMOUS;
using hector_uav_msgs::CONTROL_REMOTE;
using hector_uav_msgs::CONTROL_JOYSTICK;

class RTT_EXPORT ControllerTask : public RTT::TaskContext, public ModuleContainer
{
public:
  typedef boost::shared_ptr<BaseController> ControllerPtr;
  typedef std::vector<ControllerPtr> Controllers;

  ControllerTask(const std::string &name = "Controller");
  virtual ~ControllerTask();

  virtual void reset();
  virtual const uxvcos::Time& getTimestamp() { return _timestamp; }
  virtual const uxvcos::Time& setTimestamp(const uxvcos::Time& _timestamp);
  virtual double getDt() { return _dt; }

  ControllerTask& setControlSource(ControlSource source) { _controlSource = source; return *this; }
  ControlSource getControlSource() const { return _controlSource; }

  const Controllers &controllers() const { return _controllers; }
  ControllerPtr getController(const std::string& name) const;

  boost::shared_ptr<RC> getRC() const { return rc; }

  template <typename ControllerType> boost::shared_ptr<ControllerType> addController(ControllerType *c, bool autostart)
  {
    c->autostart(autostart);
    return addController(c);
  }

  template <typename ControllerType> boost::shared_ptr<ControllerType> addController(ControllerType *c)
  {
    boost::shared_ptr<ControllerType> controller = addModule(c);
    _controllers.push_back(boost::shared_static_cast<BaseController>(controller));
    return controller;
  }

protected:
  virtual bool beforeExecuteHook() { return true; }
  virtual void afterExecuteHook()  {}
  virtual void resetHook()         {}
  virtual bool beforeStart()       { return true; }
  virtual void afterStop()         {}
  
  bool configureHook();
  bool startHook();
  void updateHook();
  void stopHook();
  void cleanupHook();

  boost::shared_ptr<RC> rc;
//  RTT::InputPort<rosgraph_msgs::Clock> portClock;

private:
  ControlSource _controlSource;
  uxvcos::Time _timestamp;
  double _dt;
  double _latency;
  Controllers _controllers;
};

} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_CONTROLLERTASK_H
