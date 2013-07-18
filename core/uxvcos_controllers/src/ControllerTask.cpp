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
