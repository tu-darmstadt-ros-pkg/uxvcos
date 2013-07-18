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
