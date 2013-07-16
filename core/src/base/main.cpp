//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include <base/Application.h>
#include <base/Configuration.h>

#include <rtt/TaskContext.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/scripting/ScriptingService.hpp>
#include <rtt/scripting/StateMachine.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <ocl/DeploymentComponent.hpp>

#include <boost/filesystem/path.hpp>

//#include <logging/Logging.h>
//#include <logging/Replay.h>
//#include <server/Server.h>

using namespace RTT;
using namespace uxvcos;
namespace bfs = boost::filesystem;

class APPLICATION : public Application {
private:
  std::string configurationPath;
  std::string scriptPath;
  std::vector<std::string> scriptFiles;

//  Logging::Logging *logging;
//  Logging::Replay *replay;
  RTT::scripting::StateMachinePtr stateMachine;

public:
  APPLICATION(const std::string& name = APPLICATION_NAME)
    : Application(APPLICATION_NAME)
//    , logging(0)
//    , replay(0)
  {
    // Logging::Replay::setup();
    // Logging::Logging::setup();
    // Connector::Server::setup();

    const char *temp = getenv("UXVCOS_CONFIGURATION");
    if (temp) configurationPath = temp;
    temp = getenv("UXVCOS_SCRIPT_PATH");
    if (temp) {
      scriptPath = temp;
      if (!scriptPath.empty() && scriptPath.at(scriptPath.length() - 1) != '/')
        scriptPath += "/";
    }

    // add some more program options
    this->addOption<bool>("simulation", "Start in simulation mode");
    this->addOption<unsigned>("level", "Application level:\n  1 = Raw\n  2 = Sensors (default)\n  3 = Navigation\n  4 = Controller\n  5 = Logfile\n");
    this->addOption<std::vector<std::string> >("start", scriptFiles, "Script files to load at startup");
    Options::positional().add("start", -1);
  }

  ~APPLICATION() {
  }

  bool setupHook() {
    Configuration::setStorePath(configurationPath);
    Configuration::addPath((bfs::path(configurationPath) / "common").native());
    Configuration::addPath(configurationPath);
    Configuration::addPath((bfs::path(configurationPath) / getRealm()).native());

    // import all dependent packages
#ifdef ROS_PACKAGE_NAME
    this->import(ROS_PACKAGE_NAME);
    if (getRealm() != ROS_PACKAGE_NAME)
#endif
      this->import(getRealm());

    // copied from deployer.cpp
    for (std::vector<std::string>::const_iterator iter=scriptFiles.begin();
         iter!=scriptFiles.end();
         ++iter)
    {
        if ( !(*iter).empty() )
        {
            if ( (*iter).rfind(".xml",std::string::npos) == (*iter).length() - 4 || (*iter).rfind(".cpf",std::string::npos) == (*iter).length() - 4) {
                deployment()->kickStart( scriptPath + (*iter) );
                continue;
            } if ( (*iter).rfind(".ops",std::string::npos) == (*iter).length() - 4 || (*iter).rfind(".osd",std::string::npos) == (*iter).length() - 4) {
                deployment()->runScript( scriptPath + (*iter) );
                continue;
            }
            log(Error) << "Unknown extension of file: '"<< (*iter) <<"'. Must be xml, cpf for XML files or, ops or osd for script files."<<endlog();
        }
    }

//    RTT::TaskContext *server = this->addTrigger(loadComponent("Server", "Server"));
//    if (server) {
//      RTT::TaskContext *telemetry = this->addTrigger(loadComponent("Telemetry", "Telemetry"));
//      telemetry->addPeer(server);
//    }

    // if (getOption<bool>("simulation")) this->addTrigger(loadComponent("Simulation", "Server"));

    // this->addTrigger(loadComponent("Communication", "Communication"));

    // add Logging
    //this->addTrigger(new Logging::Logging());

    // Replay has to be started last
    //this->addTask(replay);

    return true;
  }

  bool configureHook()
  {
    // set Application echelon
    if (this->hasOption("level")) {
      this->setEchelon(static_cast<EchelonType>(this->getOption<unsigned>("level")));
    }

    /*
      // start in replay mode?
      replay = new Logging::Replay();
      if (!replay || !replay->mode()) {
        delete replay;
        replay = 0;
      } else {
        this->setEchelon(replay->mode());
      }
    */

    // start in simulation mode?
    if (hasOption("simulation")) {
      if (this->getEchelon() == Echelon::None) this->setEchelon(Echelon::Sensors);
      this->getMainActivity()->setPeriod(0.0);
    }

    // read configuration files
    for(Application::Tasks::const_iterator it = getTasks().begin(); it != getTasks().end(); ++it) {
      Configuration(it->get<0>()).read();
    }

    // load main scripts and state machine
    if (scripting()) {
//      scripting()->loadPrograms(Configuration::buildFilename(scriptPath, getRealm(), "ops"));
      RTT::scripting::ProgramInterfacePtr program = scripting()->getProgram("configure");
      if (program) {
        RTT::log(RTT::Info) << "Executing program " << program->getName() << "..." << RTT::endlog();
        if (!(program->start() && program->execute())) return false;
      }

//      if (scripting()->loadStateMachines(Configuration::buildFilename(scriptPath, getRealm(), "osd"))) {
//        stateMachine = boost::dynamic_pointer_cast<RTT::scripting::ScriptingService>(root()->provides("scripting"))->getStateMachine("Main");
//      }
    }

    return true;
  }

  bool startHook()
  {
    // execute start program from scripting
    if (scripting()) {
      RTT::scripting::ProgramInterfacePtr program = scripting()->getProgram("start");
      if (program) {
        RTT::log(RTT::Info) << "Executing program " << program->getName() << "..." << RTT::endlog();
        if (!(program->start() && program->execute())) return false;
      }
    }

    // activate and start state machine Main
    if (stateMachine) {
      stateMachine->activate();
      stateMachine->start();
    }

    return true;
  }

  void runningHook()
  {
    // execute running program from scripting
    if (scripting()) {
      RTT::scripting::ProgramInterfacePtr program = scripting()->getProgram("running");
      if (program) {
        RTT::log(RTT::Info) << "Executing program " << program->getName() << "..." << RTT::endlog();
        program->start() && program->execute();
      }
    }
  }

  bool beforeStopHook()
  {
    // execute stop program from scripting
    if (scripting()) {
      RTT::scripting::ProgramInterfacePtr program = scripting()->getProgram("stop");
      if (program) {
        RTT::log(RTT::Info) << "Executing program " << program->getName() << "..." << RTT::endlog();
        if (!(program->start() && program->execute())) return false;
      }
    }

    if (stateMachine) {
      stateMachine->stop();
      stateMachine->deactivate();
    }

    return true;
  }

  void cleanupHook()
  {
    // execute cleanup program from scripting
    if (scripting()) {
      RTT::scripting::ProgramInterfacePtr program = scripting()->getProgram("cleanup");
      if (program) {
        RTT::log(RTT::Info) << "Executing program " << program->getName() << "..." << RTT::endlog();
        program->start() && program->execute();
      }
    }

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

#include "main.inl"
