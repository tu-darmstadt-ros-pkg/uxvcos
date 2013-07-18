//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
