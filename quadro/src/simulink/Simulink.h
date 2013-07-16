#ifndef QUADRO_SIMULINK_H
#define QUADRO_SIMULINK_H

#include <simulink/SimulinkInterface.h>

#include <nav_msgs/typekit/Odometry.h>
#include <sensor_msgs/typekit/Imu.h>
#include <sensor_msgs/typekit/Range.h>
#include <sensor_msgs/typekit/NavSatFix.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>
#include <hector_uav_msgs/typekit/Altimeter.h>
#include <hector_uav_msgs/typekit/Supply.h>
#include <hector_uav_msgs/typekit/RC.h>
#include <hector_uav_msgs/typekit/MotorStatus.h>

#include <hector_std_msgs/typekit/Float64.h>
#include <hector_uav_msgs/typekit/PositionXYCommand.h>
#include <hector_uav_msgs/typekit/VelocityXYCommand.h>
#include <hector_uav_msgs/typekit/AttitudeCommand.h>
#include <hector_uav_msgs/typekit/HeadingCommand.h>
#include <hector_uav_msgs/typekit/HeightCommand.h>
#include <hector_uav_msgs/typekit/ThrustCommand.h>

#include <hector_uav_msgs/typekit/MotorCommand.h>
#include <hector_uav_msgs/typekit/ServoCommand.h>
#include <hector_uav_msgs/typekit/RuddersCommand.h>

#include <hector_uav_msgs/typekit/ControllerState.h>
#include <hector_uav_msgs/typekit/MotorPWM.h>
#include <hector_std_msgs/typekit/Float64Array.h>

#include <quadro_msgs/typekit/Height.h>

namespace uxvcos {
namespace quadro {

class Simulink : public RTT::TaskContext, public uxvcos::Simulink::SimulinkInterface
{
  public:
    Simulink(const std::string &name = "Simulink", const std::string& model = "");
    virtual ~Simulink();

    static int setup();
    
  protected:
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  protected:
    struct {
      RTT::InputPort<sensor_msgs::Imu>                 portIMU;
      RTT::InputPort<nav_msgs::Odometry>               portState;

      RTT::InputPort<sensor_msgs::Imu>                 portRawIMU;
      RTT::InputPort<hector_uav_msgs::Supply>          portSupply;
      RTT::InputPort<sensor_msgs::Range>               portUltrasound;
      RTT::InputPort<hector_uav_msgs::Altimeter>       portAltimeter;
      RTT::InputPort<hector_std_msgs::Float64>         portTemperature;
      RTT::InputPort<hector_std_msgs::Float64>         portAirspeed;
      RTT::InputPort<sensor_msgs::NavSatFix>           portGPS;
      RTT::InputPort<geometry_msgs::Vector3Stamped>    portGPSVelocity;
      RTT::InputPort<geometry_msgs::Vector3Stamped>    portMagnetic;
      RTT::InputPort<hector_uav_msgs::MotorStatus>     portMotorStatus;

      RTT::InputPort<hector_uav_msgs::RC>              portRC;

      RTT::InputPort<hector_uav_msgs::PositionXYCommand>     portPositionCommand;
      RTT::InputPort<hector_uav_msgs::VelocityXYCommand>     portVelocityCommand;
      RTT::InputPort<hector_uav_msgs::AttitudeCommand>       portAttitudeCommand;
      RTT::InputPort<hector_uav_msgs::HeadingCommand>        portHeadingCommand;
      RTT::InputPort<hector_uav_msgs::HeightCommand>         portHeightCommand;
      RTT::InputPort<hector_uav_msgs::ThrustCommand>         portThrottleCommand;
      RTT::InputPort<hector_uav_msgs::RuddersCommand>        portRuddersCommand;

      RTT::InputPort<hector_uav_msgs::ControllerState> portControllerState;
      RTT::InputPort<quadro_msgs::Height>              portHeight;
      RTT::InputPort<sensor_msgs::Imu>                 portReferenceOrientation;

      RTT::InputPort<hector_uav_msgs::MotorPWM>        portMotorOutput;
    } inputs;

    struct {
      RTT::OutputPort<sensor_msgs::Imu>                portIMU;
      RTT::OutputPort<nav_msgs::Odometry>              portState;
      RTT::OutputPort<geometry_msgs::Vector3Stamped>   portLinearAccelerationBias;
      RTT::OutputPort<geometry_msgs::Vector3Stamped>   portAngularVelocityBias;

      RTT::OutputPort<hector_uav_msgs::MotorPWM>       portMotorOutput;
      RTT::OutputPort<hector_uav_msgs::ServoCommand>   portServoOutput;
      RTT::OutputPort<int>                             portControlMode;

      RTT::OutputPort<hector_std_msgs::Float64Array>   portLogOutput;
    } outputs;

    // RTT::Property<sensor_msgs:> *propertyReference;

    //RTT::Operation<bool(int,int,int)> switchModeMethod;
    //bool switchMode(int pitch, int height, int rollyaw);
};

} // namespace quadro
} // namespace uxvcos

#endif // QUADRO_SIMULINK_H
