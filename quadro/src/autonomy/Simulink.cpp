#include "Simulink.h"

#include <dlfcn.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <base/Application.h>
#include <base/SetupFunction.h>
#include <options/options.h>

#include <rtt/os/Mutex.hpp>

namespace Autonomy {

namespace { ::SetupFunction setup(&Simulink::setup, "Autonomy"); }

int Simulink::setup() {
  Options::Description options("Autonomy options");
  options.add_options()
    ("simulink-autonomy", Options::value<std::string>()->implicit_value(std::string()), "Use compiled simulink model as autonomy controller");
  Options::options().add(options);
  return 0;
}

Simulink::Simulink(const std::string &name, const std::string& modelName)
  : Autonomy(name)
  , ::Simulink::Interface(this, modelName)

  , portIMU("BiasedIMU")
  , portNavigation("Solution")
  , portLocalNavigation("LocalSolution")

  , portSupply("Supply")
  , portUltrasound("Ultrasound")
  , portBaro("Baro")
  , portTemperature("Temperature")
  , portAirspeed("Airspeed")
  , portGPS("GPS")
  , portCompass("Compass")
  , portMotorStatus("MotorStatus")

  , portRC("RC")
  , portUsedHeight("UsedHeight")
  , portControllerState("ControlState")

  , portPositionInput("PositionCommand")
  , portVelocityInput("VelocityCommand")
  , portAttitudeInput("AttitudeCommand")
  , portHeadingInput("HeadingCommand")
  , portTurnrateInput("TurnrateCommand")
  , portHeightInput("HeightCommand")
  , portClimbrateInput("ClimbrateCommand")
  , portThrustInput("ThrustCommand")
  , portMotorCommandInput("MotorOutput")

  , portPositionOutput("PositionInput")
  , portVelocityOutput("VelocityInput")
  , portAttitudeOutput("AttitudeInput")
  , portHeadingOutput("HeadingInput")
  , portTurnrateOutput("TurnrateInput")
  , portHeightOutput("HeightInput")
  , portClimbrateOutput("ClimbrateInput")
  , portThrustOutput("ThrustInput")
  , portMotorCommandOutput("MotorInput")

  , portControlMode("ControlMode")

  , propertyReference(0)
{
  this->ports()->addPort( portIMU );
  this->ports()->addPort( portNavigation );
  this->ports()->addPort( portLocalNavigation );

  this->ports()->addPort( portSupply );
  this->ports()->addPort( portUltrasound );
  this->ports()->addPort( portBaro );
  this->ports()->addPort( portTemperature );
  this->ports()->addPort( portAirspeed );
  this->ports()->addPort( portNavigation );
  this->ports()->addPort( portIMU );
  this->ports()->addPort( portGPS );
  this->ports()->addPort( portCompass );

  this->ports()->addPort( portRC );
  this->ports()->addPort( portUsedHeight );
  this->ports()->addPort( portControllerState );

  this->ports()->addPort( portPositionInput );
  this->ports()->addPort( portVelocityInput );
  this->ports()->addPort( portAttitudeInput );
  this->ports()->addPort( portHeadingInput );
  this->ports()->addPort( portTurnrateInput );
  this->ports()->addPort( portHeightInput );
  this->ports()->addPort( portClimbrateInput );
  this->ports()->addPort( portThrustInput );
  this->ports()->addPort( portMotorStatus );
  this->ports()->addPort( portMotorCommandInput );

  this->ports()->addPort( portPositionOutput );
  this->ports()->addPort( portVelocityOutput );
  this->ports()->addPort( portAttitudeOutput );
  this->ports()->addPort( portHeadingOutput );
  this->ports()->addPort( portTurnrateOutput );
  this->ports()->addPort( portHeightOutput );
  this->ports()->addPort( portClimbrateOutput );
  this->ports()->addPort( portThrustOutput );
  this->ports()->addPort( portMotorCommandOutput );

  this->ports()->addPort( portAutonomyCommand );
  this->ports()->addPort( portAutonomyState );
  this->ports()->addPort( portControlMode );

  if (::Application::Instance() && ::Application::Instance()->getTask("Navigation")) {
    propertyReference = ::Application::Instance()->getTask("Navigation")->properties()->getPropertyType<Data::Navigation::ReferencePosition>("ReferencePosition");
  }
}

Simulink::~Simulink()
{
  stop();
  cleanup();
}

bool Simulink::configureHook()
{
  RTT::Logger::In in(getName());

  // load the model
  if (!Options::variables("simulink-autonomy").empty()) {
    std::string new_model_name = Options::variables<std::string>("simulink-autonomy");
    if (!new_model_name.empty()) modelName.set(new_model_name);
  }
  if (!::Simulink::Interface::configureHook()) return false;

  // connect ports
  connectToSimulinkPort(portIMU);
  connectToSimulinkPort(portNavigation);
  connectToSimulinkPort(portLocalNavigation);
  connectToSimulinkPort(portSupply);
  connectToSimulinkPort(portUltrasound);
  connectToSimulinkPort(portBaro);
  connectToSimulinkPort(portTemperature);
  connectToSimulinkPort(portAirspeed);
  connectToSimulinkPort(portGPS);
  connectToSimulinkPort(portCompass);
  connectToSimulinkPort(portMotorStatus);

  connectToSimulinkPort(portRC);
  connectToSimulinkPort(portUsedHeight, "ControlHeight");
  connectToSimulinkPort(portControllerState, "ControlState");

  connectToSimulinkPort(portPositionInput, "PositionCommandInput");
  connectToSimulinkPort(portVelocityInput, "VelocityCommandInput");
  connectToSimulinkPort(portAttitudeInput, "CommandInput");
  connectToSimulinkPort(portHeadingInput, "HeadingCommandInput");
  connectToSimulinkPort(portTurnrateInput, "TurnrateCommandInput");
  connectToSimulinkPort(portHeightInput, "HeightCommandInput");
  connectToSimulinkPort(portClimbrateInput, "ClimbrateCommandInput");
  connectToSimulinkPort(portThrustInput, "ThrustCommandInput");
  connectToSimulinkPort(portMotorCommandInput, "MotorCommandInput");

  connectToSimulinkPort(portPositionOutput, "PositionCommandOutput");
  connectToSimulinkPort(portVelocityOutput, "VelocityCommandOutput");
  connectToSimulinkPort(portAttitudeOutput, "CommandOutput");
  connectToSimulinkPort(portHeadingOutput, "HeadingCommandOutput");
  connectToSimulinkPort(portTurnrateOutput, "TurnrateCommandOutput");
  connectToSimulinkPort(portHeightOutput, "HeightCommandOutput");
  connectToSimulinkPort(portClimbrateOutput, "ClimbrateCommandOutput");
  connectToSimulinkPort(portThrustOutput, "ThrustCommandOutput");
  connectToSimulinkPort(portMotorCommandOutput, "MotorCommandOutput");

  connectToSimulinkPort(portControlMode);

  return true;
}

bool Simulink::startHook()
{
  RTT::Logger::In in(getName());

  if (!::Simulink::Interface::startHook()) return false;
  return true;
}

void Simulink::updateHook()
{
  RTT::Logger::In in(getName());

  Data::Autonomy::State command;
  if (portAutonomyCommand.read(command) == RTT::NewData) {
    sendCommand(command.state);
  }

  ::Simulink::Interface::updateHook();

  int temp;
  if (hasOutputPort("AutonomyState") && getOutputPort<int>("AutonomyState")->read(temp) == RTT::NewData) {
    if (current_state.state != static_cast<unsigned int>(temp)) {
      current_state.state = static_cast<unsigned int>(temp);
      current_state.setTimestamp(getTimestamp());
      RTT::log(RTT::Info) << "Current autonomy state is " << current_state.state << "." << RTT::endlog();
      portAutonomyState.write(current_state);
    }
  }

  // show ActiveState
  if (hasOutputPort("ActiveState") && getOutputPort<int>("ActiveState")->read(temp) == RTT::NewData) {
    unsigned int currentState = temp;
    static unsigned int oldState = currentState;
    unsigned int changed = oldState ^ currentState;
    if (changed) {
      std::stringstream ss;
      for (int i = 0; i < 15; ++i) ss << ((currentState & (1 << i)) ? 1 : 0) << " ";
      RTT::log(RTT::Info) << "ActiveState changed to " << ss.str() << RTT::endlog();
    }
    oldState = currentState;
  }

  std::vector<double> debug;
  if (hasOutputPort("Debug") && getOutputPort<std::vector<double> >("Debug")->read(debug) == RTT::NewData) {
    static std::vector<double> oldDebug;
    if (debug != oldDebug) {
      RTT::log(RTT::Info) << "Debug = [";
      for(size_t i = 0; i < debug.size(); ++i) RTT::log() << debug[i] << ",";
      RTT::log() << "]" << RTT::endlog();
    }
    oldDebug = debug;
  }

  // signal waiting operations
  updateMutex.unlock();
}

void Simulink::sendCommand(unsigned int command) {
  if (hasInputPort("AutonomyCommand")) {
    getInputPort<int>("AutonomyCommand")->write(command);
    RTT::log(RTT::Info) << "Sending autonomy command " << command << RTT::endlog();
  }
}

void Simulink::reset() {
  Autonomy::reset();
}

void Simulink::stopHook()
{
  RTT::Logger::In in(getName());
  ::Simulink::Interface::stopHook();
}

void Simulink::cleanupHook()
{
  RTT::Logger::In in(getName());
  ::Simulink::Interface::cleanupHook();
}

} // namespace Controller

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( Autonomy::Simulink )
