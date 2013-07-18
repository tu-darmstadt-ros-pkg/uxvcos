#define APPLICATION FMP
#include <uxvcos/Application.h>
#include <uxvcos/Configuration.h>

#include <rtt/TaskContext.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <ros/console.h>

using namespace RTT;
using namespace uxvcos;

class FMP : public Application {
private:
  bool simulation;

public:
  FMP()
    : Application("FMP")
    , simulation(false)
  {
    this->addOption<bool>("simulation", simulation, "Start in simulation mode");
    this->addOption<unsigned>("level", "Application level:\n  1 = Raw\n  2 = Sensors (default)\n  3 = Navigation\n  4 = Controller\n  5 = Logfile\n");
  }

  ~FMP() {
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

      RTT::TaskContext *fmpbox = this->addTask(loadComponent("FMPBox", "Interface::FMPBox", false, Base::Sensors));
      if (fmpbox) fmpbox->setActivity(new RTT::Activity(RTT::os::HighestPriority, 0, "FMPBox"));

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

    return true;
  }


  void cleanupHook()
  {
    // save configuration
    for(Application::Tasks::const_iterator it = getTasks().begin(); it != getTasks().end(); ++it) {
      Configuration(it->get<0>()).store("", "", "last");
    }
  }
};

#include <uxvcos/main.inl>
