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

#include "simulink/SimulinkInterface.h"

#include <dlfcn.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rtt/extras/SlaveActivity.hpp>

namespace uxvcos {
namespace Simulink {

SimulinkInterface::SimulinkInterface(RTT::TaskContext *owner, const std::string& modelName)
  : owner(owner)
  , model(0)
  , dlhandle(0)
  , model_mtime(0)
  , modelName("Model", "Name of the Simulink model to run", modelName)
{
  owner->addPort("clock", portClock);
  owner->provides()->addOperation("reset", &SimulinkInterface::reset, this, RTT::OwnThread);
  owner->properties()->addProperty(this->modelName);
}

SimulinkInterface::~SimulinkInterface()
{
}
  
bool SimulinkInterface::loadLibrary(const std::string& modelname)
{
  char* error = 0;
  model_name = modelname;
  if (model_name.empty()) model_name = modelName.get();
  if (model_name.empty()) {
    RTT::log(RTT::Error) << "No model specified." << error << RTT::endlog();
    return false;
  }
  
  if (dlhandle != 0) {
    RTT::log( RTT::Warning ) << "Simulink controller library already loaded!" << RTT::endlog();
    unloadLibrary();
  }
  
  dlerror();    /* Clear any existing error */

  model_filename = model_name;
  dlhandle = dlopen(model_filename.c_str(), RTLD_NOW | RTLD_GLOBAL);

  if (!dlhandle) {
    model_filename = "RTW" + model_name + ".so";
    dlhandle = dlopen(model_filename.c_str(), RTLD_NOW | RTLD_GLOBAL);
  }

  if (!dlhandle) {
    model_filename = "../simulink/RTW" + model_name + ".so";
    dlhandle = dlopen(model_filename.c_str(), RTLD_NOW | RTLD_GLOBAL);
  }

  if (!dlhandle) {
    dlhandle = 0;
    error = dlerror();
    RTT::log(RTT::Error) << "Could not load model '" << model_name << "': " << error << RTT::endlog();
    model_name.clear();
    model_filename.clear();
    return false;
  }

  struct stat buf;
  if (stat(model_filename.c_str(), &buf) == 0) {
    struct tm tm;
    model_mtime = buf.st_mtime;
    localtime_r(&model_mtime, &tm);
    char buffer[200];
    strftime(buffer, sizeof(buffer), "%c", &tm);
    RTT::log(RTT::Info) << "Model '" << model_name << "' last modified at " << buffer << RTT::endlog();
  }
  
  RTT::log(RTT::Info) << "Successfully loaded model '" << model_name << "'" << RTT::endlog();
  return true;
}

void SimulinkInterface::unloadLibrary() {
  dlclose(dlhandle);
  dlhandle = 0;
}

RTT::TaskContext *SimulinkInterface::loadModel(const std::string& instance) {
  char* error = 0;
  
  if (!dlhandle && !loadLibrary()) return 0;
  
  dlerror();    /* Clear any existing error */
  RTT::TaskContext* (*createComponent)(std::string) = 0;
  createComponent = (RTT::TaskContext*(*)(std::string))(dlsym(dlhandle, "createComponent") );
  if ((error = dlerror()) != NULL) {
    RTT::log(RTT::Debug) << error << RTT::endlog();
  }
  
  return (*createComponent)(instance);
}

void SimulinkInterface::unloadModel() {
  delete model;
  model = 0;
}

bool SimulinkInterface::reset()
{
  if (!model) return false;

  bool wasRunning = model->isRunning();
  model->stop();
  model->cleanup();
  if (!model->configure()) return false;
  if (wasRunning) {
    if (!model->start()) return false;
  }

  return true;
}

bool SimulinkInterface::configureHook()
{
  if (model) cleanupHook();
  if (!loadLibrary() || !(model = loadModel())) return false;
   
  model->setActivity(new RTT::extras::SlaveActivity(owner->getPeriod()));
  if (!model->configure()) return false;

  // clone Simulink ports and connect to the events of the cloned ports
  RTT::DataFlowInterface::Ports ports = model->ports()->getPorts();
  for(RTT::DataFlowInterface::Ports::iterator it = ports.begin(); it != ports.end(); ++it) {
    RTT::base::InputPortInterface* inputPort = dynamic_cast<RTT::base::InputPortInterface*>(*it);
    if (inputPort) {
      if (inputPorts.count(inputPort->getName()) != 0) {
        RTT::log(RTT::Error) << "Model has two or more InputPorts with the same name: " << inputPort->getName() << RTT::endlog();
        cleanupHook();
        return false;
      }
      RTT::base::OutputPortInterface* outputPort = static_cast<RTT::base::OutputPortInterface*>(inputPort->antiClone());
      if (!outputPort || !outputPort->connectTo(inputPort, RTT::ConnPolicy::data(RTT::ConnPolicy::UNSYNC))) {
        delete outputPort;
        continue;
      }

      inputPorts[inputPort->getName()] = outputPort;
    }

    RTT::base::OutputPortInterface* outputPort = dynamic_cast<RTT::base::OutputPortInterface*>(*it);
    if (outputPort) {
      if (outputPorts.count(outputPort->getName()) != 0) {
        RTT::log(RTT::Error) << "Model has two or more OutputPorts with the same name:" << outputPort->getName() << RTT::endlog();
        cleanupHook();
        return false;
      }
      RTT::base::InputPortInterface* inputPort = static_cast<RTT::base::InputPortInterface*>(outputPort->antiClone());
      if (!inputPort || !outputPort->connectTo(inputPort, RTT::ConnPolicy::data(RTT::ConnPolicy::UNSYNC))) {
        delete inputPort;
        continue;
      }

      outputPorts[outputPort->getName()] = inputPort;
    }
  }

  // connect ports
  if (!connectPorts()) { /* return false; */ }

  // check if there are unconnected ports
  unconnectedPorts();

  // add model as a peer
  owner->addPeer(model);

  return true;
}

bool SimulinkInterface::connectPorts()
{
  bool result = true;

  // connect input ports
  for(InputPorts::iterator it = inputPorts.begin(); it != inputPorts.end(); ++it) {
    RTT::base::OutputPortInterface *simulink = it->second;
    if (!externalInputPorts.count(simulink->getName())) {
      RTT::log(RTT::Error) << "Did not find a matching InputPort for '" << simulink->getName() << "'" << RTT::endlog();
      result = false;
      continue;
    }

    RTT::base::InputPortInterface *external = externalInputPorts.at(simulink->getName()).get<0>();
    if (!externalInputPorts.at(simulink->getName()).get<1>()(simulink)) {
      result = false;
      continue;
    }

    if (!owner->ports()->getPort(external->getName())) {
      owner->ports()->addPort(*external);
    }
  }

  // connect output ports
  for(OutputPorts::iterator it = outputPorts.begin(); it != outputPorts.end(); ++it) {
    RTT::base::InputPortInterface *simulink = it->second;
    if (!externalOutputPorts.count(simulink->getName())) {
      RTT::log(RTT::Error) << "Did not find a matching OutputPort for '" << simulink->getName() << "'" << RTT::endlog();
      result = false;
      continue;
    }

    RTT::base::OutputPortInterface *external = externalOutputPorts.at(simulink->getName()).get<0>();
    if (!externalOutputPorts.at(simulink->getName()).get<1>()(simulink)) {
      result = false;
      continue;
    }

    if (!owner->ports()->getPort(external->getName())) {
      owner->ports()->addPort(*external);
    }
  }

  return result;
}

bool SimulinkInterface::startHook()
{
  if (!model) return false;

  struct stat buf;
  if (stat(model_filename.c_str(), &buf) == 0) {
    if (model_mtime != buf.st_mtime) {
      RTT::log(RTT::Info) << "Model '" << model_name << "' has changed, reconfiguring... " << RTT::endlog();
      if (!owner->configure()) return false;
    }
  }

  RTT::log(RTT::Info) << "Starting " << model->getName() << RTT::endlog();
  if (!model->start()) return false;
  
  return true;
}

void SimulinkInterface::updateHook()
{
  RTT::Logger::In in(owner->getName());

  // read timestamp
  rosgraph_msgs::Clock temp;
  if (portClock.read(temp) == RTT::NewData) {
    RTT::log(RTT::Debug) << "Updated clock to t = " << temp.clock << RTT::endlog();
    setTimestamp(temp.clock);
  }

  // update input ports
  for(Connections::iterator it = inputConnections.begin(); it != inputConnections.end(); ++it) {
    RTT::DataFlowInterface::SlotFunction& callback = it->second;
    if (callback) callback(it->first);
  }

  // run the model
  RTT::log(RTT::Debug) << "Updating model" << RTT::endlog();
  model->update();

  // update output ports
  for(Connections::iterator it = outputConnections.begin(); it != outputConnections.end(); ++it) {
    RTT::DataFlowInterface::SlotFunction& callback = it->second;
    if (callback) callback(it->first);
  }
}

void SimulinkInterface::stopHook()
{
  if (!model) return;
  RTT::log(RTT::Info) << "Stopping " << model->getName() << RTT::endlog();
  model->stop();
}

void SimulinkInterface::cleanupHook()
{
  // clear model input ports
  for(InputPorts::iterator it = inputPorts.begin(); it != inputPorts.end(); ++it) {
    it->second->disconnect();
    delete it->second;
  }
  inputConnections.clear();
  inputPorts.clear();

  // clear model output ports
  for(OutputPorts::iterator it = outputPorts.begin(); it != outputPorts.end(); ++it) {
    it->second->disconnect();
    delete it->second;
  }
  outputConnections.clear();
  outputPorts.clear();

  // remove input ports from the external interface
  for(ExternalInputPorts::iterator it = externalInputPorts.begin(); it != externalInputPorts.end(); ++it) {
    RTT::base::InputPortInterface *port = it->second.get<0>();
    // <-- stay connected for later configurations!
    // owner->ports()->removePort(port->getName());
  }

  // remove output ports from the external interface
  for(ExternalOutputPorts::iterator it = externalOutputPorts.begin(); it != externalOutputPorts.end(); ++it) {
    RTT::base::OutputPortInterface *port = it->second.get<0>();
    // <-- stay connected for later configurations!
    // owner->ports()->removePort(port->getName());
  }

  // cleanup the Simulink model
  if (model) {
    // model->cleanup();
    owner->removePeer(model);
  }
  
  // unload the model and library
  unloadModel();
  unloadLibrary();
}

bool SimulinkInterface::unconnectedPorts() {
  bool result = true;

  // check input ports
  for(InputPorts::iterator it = inputPorts.begin(); it != inputPorts.end(); ++it) {
    RTT::base::OutputPortInterface *simulink = it->second;
    if (!simulink->connected()) {
      RTT::log(RTT::Warning) << "Model input port '" << simulink->getName() << "' is currently not connected." << RTT::endlog();
      result = false;
    }
  }

  // check output ports
  for(OutputPorts::iterator it = outputPorts.begin(); it != outputPorts.end(); ++it) {
    RTT::base::InputPortInterface *simulink = it->second;
    if (!simulink->connected()) {
      RTT::log(RTT::Warning) << "Model output port '" << simulink->getName() << "' is currently not connected." << RTT::endlog();
      result = false;
    }
  }

  return result;
}

} // namespace Simulink
} // namespace uxvcos
