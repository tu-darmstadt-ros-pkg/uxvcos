#ifndef INTERFACE_MOTOR_H
#define INTERFACE_MOTOR_H

#include <base/Module.h>
#include <base/Trigger.h>
#include <rtt/Logger.hpp>

#include <hector_uav_msgs/typekit/MotorPWM.h>

namespace uxvcos {
namespace Interface {

class Motor : public Module {
public:
  class Handler {
    public:
      virtual ~Handler() {}
      virtual bool newMotorCommand(const hector_uav_msgs::MotorPWM& motorCommand) = 0;
  };

  Motor(RTT::TaskContext* parent, const std::string& name = "Motor", const std::string& description = "Interface to the Motors")
    : Module(parent, name, description)
    , command("motor_output")
    , handler(dynamic_cast<Handler*>(parent))
  {
    RTT::Logger::In in("Motor");

    command.doc("PWM outputs");
    this->addPort(command);
    parent->addPort(command);

    if (!handler) RTT::log( RTT::Error ) << "No Motor handler defined in " << __FUNCTION__ << RTT::endlog();

    trigger = boost::bind(&Motor::newMotorCommand, this, _1);
    trigger.addPort(command);
  }

  virtual ~Motor() {
    parent->removePort(command.getName());
  }

  void newMotorCommand(RTT::base::PortInterface* port) {
    if (!handler || port != &command) return;

    if (RTT::NewData == command.read(data))
      handler->newMotorCommand(data);
  }

  const hector_uav_msgs::MotorPWM& getMotorCommand() const {
    return data;
  }

public:
  RTT::InputPort <hector_uav_msgs::MotorPWM> command;
  Trigger trigger;

protected:
  Handler *handler;
  hector_uav_msgs::MotorPWM data;
};

} // namespace Interface
} // namespace uxvcos


#endif // INTERFACE_MOTOR_H
