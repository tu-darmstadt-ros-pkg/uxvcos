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
