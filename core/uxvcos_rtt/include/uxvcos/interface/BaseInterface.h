#ifndef INTERFACE_BASEINTERFACE_H
#define INTERFACE_BASEINTERFACE_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <uxvcos/ModuleContainer.h>

#include <uxvcos/Time.h>
#include <rosgraph_msgs/typekit/Clock.h>

namespace uxvcos {
namespace Interface {

class RTT_EXPORT BaseInterface : public RTT::TaskContext, public ModuleContainer
{
public:
  BaseInterface(const std::string &name = "Interface");
  virtual ~BaseInterface();

  virtual Time getTimestamp() const {
    return clock.clock;
  }

  virtual void setTimestamp(const Time& timestamp);

protected:
  rosgraph_msgs::Clock clock;
  RTT::OutputPort<rosgraph_msgs::Clock> portClock;
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_BASEINTERFACE_H
