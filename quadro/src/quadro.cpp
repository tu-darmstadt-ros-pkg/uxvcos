#define APPLICATION Quadro
#include <base/Application.h>
#include <base/Configuration.h>
#include <base/DataPool.h>

#include <rtt/TaskContext.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <rtt/scripting/StateMachine.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <ros/console.h>

using namespace RTT;
using namespace uxvcos;

class Quadro : public Application {
private:
  RTT::scripting::StateMachinePtr stateMachine;
  bool simulation;

public:
  Quadro()
    : Application("Quadro")
    , simulation(false)
  {
    this->addOption<bool>("simulation", simulation, "Start in simulation mode");
    this->addOption<unsigned>("level", "Application level:\n  1 = Raw\n  2 = Sensors (default)\n  3 = Navigation\n  4 = Controller\n  5 = Logfile\n");
  }

  ~Quadro() {
  }

  bool setupHook() {
    Configuration::setStorePath("../conf");
    Configuration::addPath("../conf/common");
    Configuration::addPath("../conf");
    Configuration::addPath("../conf/" + getRealm());

    try {
      // set Application echelon?
      if (hasOption("level")) {
        this->setEchelon(static_cast<EchelonType>(getOption<unsigned>("level")));
      }

      // start in simulation mode?
      if (hasOption("simulation")) {
        simulation = true;
        // set Application echelon
        if (this->getEchelon() == Echelon::None) this->setEchelon(Echelon::Sensors);
        this->getMainActivity()->setPeriod(0.0);
      }

      // import components from this package and dependent packages
      this->import(ROS_PACKAGE_NAME);

      this->addTask(loadComponent("Interface", "EthernetInterface", true, Echelon::Sensors));
      this->addTask(loadComponent("Navigation", "Navigation", true, Echelon::Navigation));
      this->addTask(loadComponent("GPS", "GPS", false, Echelon::Sensors));

      if (hasOption("simulink-controller")) {
        this->addTask(loadComponent("SimulinkController", "Simulink", true, Echelon::Controller));
      }

      if (hasOption("simulink-autonomy")) {
        this->addTask(loadComponent("SimulinkAutonomy", "Simulink", true, Echelon::Controller));
      }

      #if defined(TARGET_QUADRO)
        this->addTask(loadComponent("Controller", "Controller", true, Echelon::Controller));
      #elif defined(TARGET_AIRPLANE)
        //this->addTask(loadComponent("Controller", "Controller", true, Echelon::Controller));
      #endif

      this->addTask(loadComponent("Testbed", "Testbed", false, Echelon::Sensors));

//      RTT::TaskContext *server = this->addTrigger(loadComponent("Server", "Server"));
//      if (server) {
//        RTT::TaskContext *telemetry = this->addTrigger(loadComponent("Telemetry", "Telemetry"));
//        telemetry->addPeer(server);
//      }

      // if (simulation) this->addTrigger(loadComponent("Simulation", "Server"));

      // this->addTrigger(loadComponent("Communication", "Communication"));

    } catch (std::runtime_error& e) {
      log(Fatal) << e.what() << endlog();
      return false;
    }

    return true;
  }

  bool configureHook()
  {
    // read configuration files
    for(Application::Tasks::const_iterator it = getTasks().begin(); it != getTasks().end(); ++it) {
      Configuration(it->get<0>()).read();
    }

    // load main scripts and state machine
    if (scripting()) {
      scripting()->loadPrograms(Configuration::buildFilename("../conf", getRealm(), "ops"), false);
      scripting()->startProgram("configure");

      if (scripting()->loadStateMachines(Configuration::buildFilename(getRealm(), "osd"), false)) {
        stateMachine = boost::dynamic_pointer_cast<RTT::scripting::ScriptingService>(root()->provides("scripting"))->getStateMachine("Main");
      }
    }

    return true;
  }

  bool startHook()
  {
    // run start program from scripting
    if (scripting()) scripting()->startProgram("start");

    // activate and start state machine Main
    if (stateMachine) {
      stateMachine->activate();
      stateMachine->start();
    }

    // Setup ROS streams
    RTT::ServicePtr ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<RTT::ConnPolicy(std::string)> topic;
    if (ros && (topic = ros->getOperation("topic")).ready()) {
      DataPool::Connection* connection = 0;

      if (!simulation) {

        if ((connection = DataPool::Global()->find("clock"))) {
          connection->outputPort->createStream(topic("clock"));
        }

        if ((connection = DataPool::Global()->find("raw_imu")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("raw_imu"));
        }

        if ((connection = DataPool::Global()->find("magnetic")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("magnetic"));
        }

        if ((connection = DataPool::Global()->find("altimeter")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("altimeter"));
        }

        if ((connection = DataPool::Global()->find("pressure_height")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("pressure_height"));
        }

        if ((connection = DataPool::Global()->find("supply")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("supply"));
        }

        if ((connection = DataPool::Global()->find("rc")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("rc"));
        }

        if ((connection = DataPool::Global()->find("motor_status")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("motor_status"));
        }

        if ((connection = DataPool::Global()->find("sonar_height")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("sonar_height"));
        }

        if ((connection = DataPool::Global()->find("fix")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("fix"));
        }

        if ((connection = DataPool::Global()->find("fix_velocity")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("fix_velocity"));
        }

        if ((connection = DataPool::Global()->find("temperature")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("temperature"));
        }

        if ((connection = DataPool::Global()->find("airspeed")) && connection->hasWriters()) {
          connection->outputPort->createStream(topic("airspeed"));
        }

      } else { // simulation
        if ((connection = DataPool::Global()->find("clock"))) {
          connection->inputPort->createStream(topic("clock"));
        }

        if ((connection = DataPool::Global()->find("raw_imu"))) {
          connection->inputPort->createStream(topic("raw_imu"));
        }

        if ((connection = DataPool::Global()->find("magnetic"))) {
          connection->inputPort->createStream(topic("magnetic"));
        }

        if ((connection = DataPool::Global()->find("altimeter"))) {
          connection->inputPort->createStream(topic("altimeter"));
        }

        if ((connection = DataPool::Global()->find("pressure_height"))) {
          connection->inputPort->createStream(topic("pressure_height"));
        }

        if ((connection = DataPool::Global()->find("supply"))) {
          connection->inputPort->createStream(topic("supply"));
        }

        if ((connection = DataPool::Global()->find("rc"))) {
          connection->inputPort->createStream(topic("rc"));
        }

        if ((connection = DataPool::Global()->find("motor_status"))) {
          connection->inputPort->createStream(topic("motor_status"));
        }

        if ((connection = DataPool::Global()->find("sonar_height"))) {
          connection->inputPort->createStream(topic("sonar_height"));
        }

        if ((connection = DataPool::Global()->find("fix"))) {
          connection->inputPort->createStream(topic("fix"));
        }

        if ((connection = DataPool::Global()->find("fix_velocity"))) {
          connection->inputPort->createStream(topic("fix_velocity"));
        }

        if ((connection = DataPool::Global()->find("temperature"))) {
          connection->inputPort->createStream(topic("temperature"));
        }

        if ((connection = DataPool::Global()->find("airspeed"))) {
          connection->inputPort->createStream(topic("airspeed"));
        }
      }

      if ((connection = DataPool::Global()->find("state"))) {
        if (connection->hasWriters())
          connection->outputPort->createStream(topic("state"));
        else
          connection->inputPort->createStream(topic("state"));
      }

      if ((connection = DataPool::Global()->find("imu")) && connection->hasWriters()) {
        if (connection->hasWriters())
          connection->outputPort->createStream(topic("imu"));
        else
          connection->inputPort->createStream(topic("imu"));
      }

      if ((connection = DataPool::Global()->find("global")) && connection->hasWriters()) {
        if (connection->hasWriters())
          connection->outputPort->createStream(topic("global"));
        else
          connection->inputPort->createStream(topic("global"));
      }

      if ((connection = DataPool::Global()->find("command_velocity")) && connection->hasReaders()) {
        connection->inputPort->createStream(topic("cmd_vel"));
      }

      if ((connection = DataPool::Global()->find("height")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("height"));
      }

      if ((connection = DataPool::Global()->find("force_simulator")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("force"));
      }

      if ((connection = DataPool::Global()->find("motor_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/motor"));
      }

      if ((connection = DataPool::Global()->find("motor_output")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("motor_pwm"));
      }

      if ((connection = DataPool::Global()->find("attitude_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/attitude"));
      }

      if ((connection = DataPool::Global()->find("velocity_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/velocity"));
      }

      if ((connection = DataPool::Global()->find("position_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/position"));
      }

      if ((connection = DataPool::Global()->find("heading_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/heading"));
      }

      if ((connection = DataPool::Global()->find("heading_rate_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/turnrate"));
      }

      if ((connection = DataPool::Global()->find("height_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/height"));
      }

      if ((connection = DataPool::Global()->find("height_rate_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/climbrate"));
      }

      if ((connection = DataPool::Global()->find("command_velocity_input")) && connection->hasWriters()) {
        connection->outputPort->createStream(topic("input/cmd_vel"));
      }

    } else {
      ROS_ERROR("Could not find service ros! Aborting...");
      return false;
    }

    return true;
  }

  void runningHook()
  {
    // run start program from scripting
    if (scripting()) scripting()->startProgram("poststart");
  }

  bool beforeStopHook()
  {
    // run stop program from scripting
    if (scripting()) scripting()->startProgram("stop");

    if (stateMachine) {
      stateMachine->stop();
      stateMachine->deactivate();
    }

    return true;
  }

  void cleanupHook()
  {
    // save configuration
    for(Application::Tasks::const_iterator it = getTasks().begin(); it != getTasks().end(); ++it) {
      Configuration(it->get<0>()).store("", "", "last");
    }

    // unload main state machine
    if (stateMachine) {
      stateMachine.reset();
      scripting()->unloadStateMachine("Main");
    }
  }
};

#include <base/main.inl>
