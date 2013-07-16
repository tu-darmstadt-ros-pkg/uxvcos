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
