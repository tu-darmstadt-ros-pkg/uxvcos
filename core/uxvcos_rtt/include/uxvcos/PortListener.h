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

#ifndef UXVCOS_PORTLISTENER_H
#define UXVCOS_PORTLISTENER_H

#include <rtt/DataFlowInterface.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Handle.hpp>
#include <rtt/os/Mutex.hpp>

#include <algorithm>
#include <boost/tuple/tuple.hpp>

#include <uxvcos/DataPool.h>

#include <vector>
#include <map>

#include <data/uxvcos-types.h>

namespace uxvcos {

class PortListener : public RTT::DataFlowInterface {
public:
  using RTT::DataFlowInterface::Ports;
  typedef std::vector<RTT::base::InputPortInterface *> InputPorts;
  typedef std::vector<RTT::base::OutputPortInterface *> OutputPorts;
  typedef std::map<RTT::base::OutputPortInterface *, RTT::base::InputPortInterface *> Listeners;

  PortListener(RTT::TaskContext* owner = 0, bool trigger = false, RTT::ConnPolicy policy = RTT::ConnPolicy::data()) // : RTT::Service("PortListener", owner),
    : RTT::DataFlowInterface(owner ? owner->provides().get() : 0)
    , addOutputPortOperation("addOutputPort", &PortListener::addOutputPort, this, RTT::OwnThread)
    , mtrigger(trigger)
    , policy(policy)
  {
    if (owner) addOutputPortOperation.setOwner(owner->engine());
    useDataPool(DataPool::Global());
  }

  virtual ~PortListener() {
    stop();
  }

  void useDataPool(DataPool *dataPool) {
    this->dataPool = dataPool ? dataPool : DataPool::Global();
  }

  void start() {
    if (!dataPool) return;
    addPorts(dataPool->getOutputPorts());
    if (dataPool) dataPoolHandle = dataPool->getNewDataEvent()->connect(boost::bind(&PortListener::newData, this, _1));
  }

  void stop() {
    if (dataPoolHandle.connected()) dataPoolHandle.disconnect();
    clear();
  }

  void clear() {
    // clear the DataFlowInterface (removes the services)
    RTT::DataFlowInterface::clear();

    // for(std::vector<RTT::Handle>::iterator it = handles.begin(); it != handles.end(); ++it) it->disconnect();
    for(Listeners::const_iterator it = listeners.begin(); it != listeners.end(); ++it) delete it->second;
    listeners.clear();
    ports.clear();
  }

  RTT::base::InputPortInterface *addInputPort(RTT::base::InputPortInterface *port) {
    if (!port) return 0;

    // check for duplicates
    InputPorts::iterator dup = std::find(ports.begin(), ports.end(), port);
    if (dup != ports.end()) return *dup;
    ports.push_back(port);

    // handles.push_back(port->getNewDataOnPortEvent()->connect(boost::bind(&PortListener::dataOnPort, this, _1)));
    if (mtrigger)
      addLocalEventPort(*port);
    else
      addLocalPort(*port);

    RTT::log(RTT::Debug) << (getOwner() ? getOwner()->getName() : "Someone") << " listens to " << port->getName() << " (" << port->getTypeInfo()->getTypeName() << ")" << RTT::endlog();
    return port;
  }

  RTT::base::InputPortInterface *addOutputPort(RTT::base::OutputPortInterface *port) {
    if (!port) return 0;

    // check for duplicates
    if (listeners.count(port) != 0) return listeners[port];

    RTT::base::InputPortInterface *input = static_cast<RTT::base::InputPortInterface *>(port->antiClone());
    if (!port->createConnection(*input, policy)) {
      RTT::log(RTT::Error) << "Could not connect to output port " << port->getName() << RTT::endlog();
      delete input;
      return 0;
    }
    listeners[port] = input;

    return addInputPort(input);
  }

  RTT::base::InputPortInterface *addPort(RTT::base::PortInterface *port) {
    RTT::base::InputPortInterface *inputPort   = dynamic_cast<RTT::base::InputPortInterface *>(port);
    if (inputPort)  return addInputPort(inputPort);
    RTT::base::OutputPortInterface *outputPort = dynamic_cast<RTT::base::OutputPortInterface *>(port);
    if (outputPort) return addOutputPort(outputPort);
    return 0;
  }

  RTT::base::InputPortInterface *addPort(const std::string& name) {
    if (!dataPool) return 0;
    return addPort(dataPool->getOutputPort(name));
  }

  bool addPorts(const RTT::DataFlowInterface::Ports& ports) {
    bool result = true;
    for(RTT::DataFlowInterface::Ports::const_iterator it = ports.begin(); it != ports.end(); it++) {
      if (!addPort(*it)) result = false;
    }
    return result;
  }

  RTT::base::InputPortInterface *getPort(const std::string &name) const {
    return dynamic_cast<RTT::base::InputPortInterface *>(this->DataFlowInterface::getPort(name));
  }

  const InputPorts& getPorts() const {
    return ports;
  }

  template <typename T>
  RTT::FlowStatus read(const std::string& name, T& data, bool copy_old_data = true) {
    RTT::InputPort<T> *port = getPortType<RTT::InputPort<T> >(name);
    if (!port) return RTT::NoData;
    return port->read(data, copy_old_data);
  }

//  void dataOnPort(RTT::base::PortInterface* porti)
//  {
//    RTT::base::InputPortInterface* port = static_cast<RTT::base::InputPortInterface*>(porti);
//    listenerHook(port);
//    if (mtrigger && getOwner()) getOwner()->trigger();
//  }

  void newData(DataPool::Connection *connection) {
    RTT::base::OutputPortInterface *output = connection->outputPort;
    listenerHook(connection);
    if (listeners.count(output) == 0) addOutputPortOperation.getOperationCaller()->send(output);
  }

protected:
  // virtual void listenerHook(RTT::base::InputPortInterface *) {}
  virtual void listenerHook(DataPool::Connection *) {}

protected:
  RTT::Operation<bool(RTT::base::OutputPortInterface *port)> addOutputPortOperation;
  bool mtrigger;

  DataPool *dataPool;
  RTT::Handle dataPoolHandle;
  RTT::ConnPolicy policy;

  InputPorts ports;
  Listeners listeners;
  // std::vector<RTT::Handle> handles;

private:
  RTT::os::Mutex mutex;
};

} // namespace uxvcos

#endif // UXVCOS_PORTLISTENER_H
