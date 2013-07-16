#ifndef CONTROLLER_AIRPLANE_H
#define CONTROLLER_AIRPLANE_H

#include <controller/Controller.h>
#include <controller/PID.h>
#include <controller/RC.h>
#include <controller/ControllerTask.h>

#include <base/Transformation.h>

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
