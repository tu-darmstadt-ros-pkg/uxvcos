#include "DataPool.h"
#include "Application.h"

#include <uxvcos/data/TypeInfo.h>

#include <rtt/extras/SequentialActivity.hpp>

using namespace RTT;

namespace uxvcos {

DataPool *DataPool::global = 0;

DataPool *DataPool::Global()
{
   if (!global) {
     global = new DataPool();
     // global->start();
   }
   return global;
 }

DataPool::Connection::Connection(DataPool *owner, RTT::base::PortInterface *port, std::string name, Data::Key key)
  : name(name), typeInfo(port->getTypeInfo()), key(key)
  , inputPort(static_cast<RTT::base::InputPortInterface *>(dynamic_cast<RTT::base::InputPortInterface *>(port) ? port->clone() : port->antiClone()))
  , internalInputPort(static_cast<RTT::base::InputPortInterface *>(inputPort->clone()))
  , outputPort(static_cast<RTT::base::OutputPortInterface *>(dynamic_cast<RTT::base::OutputPortInterface *>(port) ? port->clone() : port->antiClone()))
  , internalOutputPort(static_cast<RTT::base::OutputPortInterface *>(outputPort->clone()))
  , data(typeInfo->buildValue())
  , dataPool(owner)
{
  dataPool->inputPorts.addLocalEventPort(*inputPort, boost::bind(&Connection::dataOnPort, this, _1));
  dataPool->internalInputPorts.addLocalEventPort(*internalInputPort, boost::bind(&Connection::dataOnPort, this, _1));
  dataPool->outputPorts.addLocalPort(*outputPort);
  dataPool->internalOutputPorts.addLocalPort(*internalOutputPort);
}

DataPool::Connection::~Connection() {
  dataPool->inputPorts.removePort(inputPort->getName());
  dataPool->internalInputPorts.removePort(internalInputPort->getName());
  dataPool->outputPorts.removePort(outputPort->getName());
  dataPool->internalOutputPorts.removePort(internalOutputPort->getName());

  delete inputPort;
  delete internalInputPort;
  delete outputPort;
  delete internalOutputPort;
}

bool DataPool::Connection::hasReaders() const {
  return readers.size() > 0;
}

bool DataPool::Connection::hasWriters() const {
  return writers.size() > 0;
}

std::size_t DataPool::Connection::numReaders() const {
  return readers.size();
}

std::size_t DataPool::Connection::numWriters() const {
  return writers.size();
}

void DataPool::Connection::send(RTT::base::DataSourceBase::shared_ptr ds, bool forward /* = true */) {
  if (data) {
    if (!data->update(ds.get())) return;
    ds = data;
  }

  // RTT::log(RTT::Debug) << "DataPool received a " << name << " (" << typeInfo->getTypeName() << ") message" << RTT::endlog();
  if (dataPool) dataPool->newData(this);

  outputPort->write(ds);
  if (forward) {
    RTT::log(RTT::Debug) << "DataPool forwarded a " << name << " (" << typeInfo->getTypeName() << ") message" << RTT::endlog();
    internalOutputPort->write(ds);
  }
}

void DataPool::Connection::dataOnPort(RTT::base::PortInterface *port) {
  if (port == internalInputPort) {
    if (!internalInputSource) internalInputSource.reset(internalInputPort->getDataSource());
    send(internalInputSource, /* forward = */ false);
  } else {
    RTT::log(RTT::Debug) << "DataPool received data on the external port " << name << " (" << typeInfo->getTypeName() << ")" << RTT::endlog();
    if (!inputSource) inputSource.reset(inputPort->getDataSource());
    send(inputSource, /* forward = */ true);
  }
}

bool DataPool::Connection::addPort(RTT::base::PortInterface *port) {
  RTT::base::InputPortInterface *inputPort   = dynamic_cast<RTT::base::InputPortInterface *>(port);
  RTT::base::OutputPortInterface *outputPort = dynamic_cast<RTT::base::OutputPortInterface *>(port);

  if (inputPort) {
    if (readers.count(inputPort)) {
      RTT::log(RTT::Error) << "InputPort " << inputPort->getName() << " is already a member of this connection" << RTT::endlog();
      return false;
    }
    readers.insert(inputPort);
    this->internalOutputPort->connectTo(inputPort);
    for(std::set<RTT::base::OutputPortInterface *>::iterator it = writers.begin(); it != writers.end(); ++it) (*it)->connectTo(inputPort);
    return true;
  }

  if (outputPort) {
    if (writers.count(outputPort)) {
      RTT::log(RTT::Error) << "OutputPort " << outputPort->getName() << " is already a member of this connection" << RTT::endlog();
      return false;
    }
    writers.insert(outputPort);
    outputPort->connectTo(this->internalInputPort);
    for(std::set<RTT::base::InputPortInterface *>::iterator it = readers.begin(); it != readers.end(); ++it) outputPort->connectTo(*it);
    return true;
  }

  return false;
}

void DataPool::Connection::removePort(RTT::base::PortInterface *port) {
  RTT::base::InputPortInterface *inputPort   = dynamic_cast<RTT::base::InputPortInterface *>(port);
  RTT::base::OutputPortInterface *outputPort = dynamic_cast<RTT::base::OutputPortInterface *>(port);

  if (inputPort) {
    this->internalOutputPort->disconnect(inputPort);
    inputPort->disconnect();
    readers.erase(inputPort);
  }

  if (outputPort) {
    this->internalInputPort->disconnect(outputPort);
    outputPort->disconnect();
    writers.erase(outputPort);
  }
}

const std::string& DataPool::Connection::getName() const {
  return name;
}

RTT::base::DataSourceBase::shared_ptr DataPool::Connection::getDataSource() const {
  // return outputPort->getDataSource();
  return data;
}

DataPool::DataPool(const std::string& name)
  : RTT::TaskContext(name, RTT::TaskContext::Running)
  , inputPorts(this->provides().get())
  , internalInputPorts(this->provides().get())
  , outputPorts(this->provides().get())
  , internalOutputPorts(this->provides().get())
{
  this->setActivity(new RTT::extras::SequentialActivity());
  this->addOperation("addComponent", &DataPool::addComponentByName, this, RTT::OwnThread);
  this->addOperation("removeComponent", &DataPool::removeComponentByName, this, RTT::OwnThread);
}

DataPool::~DataPool() {
  while(!connections.empty()) {
    delete connections.back();
    connections.pop_back();
  }
}

DataPool::Connection *DataPool::inject(Data::Streamable *data) {
  Data::TypeInfo *typeInfo = data->getTypeInfo();
  Data::Key key = data->getKey();

  if (!typeInfo) {
    RTT::log( RTT::Warning ) << "Received data with unknown key " << key << RTT::endlog();
    return 0;
  }

  Connection* connection = find(key, typeInfo->RTTTypeInfo());
  if (!connection) {
    RTT::log( RTT::Debug ) << "Could not find matching DataPool entry for type " << typeInfo->getTypeName() << " (" << key << ")" << RTT::endlog();
    connection = addConnection(data->getName(), typeInfo, key);
    if (!connection) return 0;
  }

  // discard any packets that are also written by any other component
  if (connection->hasWriters()) {
    // RTT::log( RTT::Debug ) << "Ignoring " << data->getTypeInfo()->getName() << " (t = " << timestamp << ")" << RTT::endlog();
    return 0;
  }

  // RTT::log( RTT::Debug ) << "Reading " << data->getTypeInfo()->getName() << " (t = " << timestamp << ")" << RTT::endlog();
  // std::cerr << *data << std::endl;

  try {
    connection->send(typeInfo->buildReference(data));
    /* if (!connection->send(typeInfo->buildReference(data))) {
      RTT::log( RTT::Warning ) << "Failed to write " << connection->getName() << " with type " << typeInfo->getName() << " (" << key << ")" << RTT::endlog();
      continue;
    } */
  } catch(std::exception &e) {
    RTT::log( RTT::Warning ) << "Failed to write " << connection->getName() << " with type " << typeInfo->getTypeName() << " (" << key << "): " << e.what() << RTT::endlog();
    return 0;
  }

  return connection;
}

bool DataPool::addComponentByName(const std::string& name) {
  TaskContext *comp = Application::Instance()->getTask(name);
  if (!comp) {
    log(Error) << "Unknown component: " << name << endlog();
    return false;
  }
  return addComponent(comp);
}

bool DataPool::addComponent(TaskContext* comp) {
  Logger::In in(getName());

  log(Debug) << "Adding component " << comp->getName() << "." << endlog();

  Ports ports = comp->ports()->getPorts();
  for (Ports::iterator it = ports.begin(); it != ports.end() ; ++it) {
    RTT::base::PortInterface *port = *it;
    if (port->connected()) {
      log(Debug) << "Ignoring port " << comp->getName() << "/" << port->getName() << ": already connected" << endlog();
      continue;
    }
    this->addPort(port);
  }
  return true;
}

bool DataPool::removeComponentByName(const std::string& name) {
  TaskContext *comp = Application::Instance()->getTask(name);
  if (!comp) {
    log(Error) << "Unknown component: " << name << endlog();
    return false;
  }
  return removeComponent(comp);
}

bool DataPool::removeComponent(TaskContext* comp) {
  Logger::In in(getName());

  Ports ports = comp->ports()->getPorts();
  for (Ports::iterator it = ports.begin(); it != ports.end() ; ++it) {
    this->removePort(*it);
  }
  return true;
}

bool DataPool::addPort(RTT::base::PortInterface* port, const std::string& p_name, Data::Key key) {
  Logger::In in(getName());

  const RTT::types::TypeInfo *typeInfo = port->getTypeInfo();

  std::string name;
  if (!p_name.empty())
    name = p_name;
  else
    name = port->getName();

  Connection *connection;
  if (key)
    connection = find(key);
  else
    connection = find(name, typeInfo);

  // found matching connection?
  if (!connection) {
    // add new connection
    connection = addConnection(port, name, key);
    if (!connection) return false;
    log(Debug) << "Created new DataPool entry for " << name << endlog();
  }

  if (!connection->addPort(port)) {
    log(Error) << "Could not add port " << name << " to the DataPool!" << endlog();
    return false;
  }

  log(Debug) << "Added port " << name << " to the DataPool (" << connection->numReaders() << " readers, " << connection->numWriters() << " writers)" << endlog();
  return true;
}

DataPool::Connection* DataPool::addConnection(RTT::base::PortInterface* port, const std::string& name, Data::Key key) {
  if (!key) {
    const Data::TypeInfo *typeInfo = Data::TypeInfo::fromRTTTypeInfo(port->getTypeInfo());
    if (typeInfo) key = typeInfo->getKey(name);
  }

  Connection *connection = new Connection(this, port, name, key);
  // addLocalPort(*connection->inputPort);
  connections.push_back(connection);
  return connection;
}

DataPool::Connection* DataPool::addConnection(const std::string& name, const RTT::types::TypeInfo *typeInfo, Data::Key key) {
  // if (!key) key = typeInfo->getKey(name);

  RTT::base::InputPortInterface* temp = typeInfo->inputPort(name);
  Connection *connection = new Connection(this, temp, name, key);
  delete temp;

  // addLocalPort(*connection->inputPort);
  connections.push_back(connection);

  return connection;
}

void DataPool::removePort(RTT::base::PortInterface* port) {
  for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
    (*it)->removePort(port);
  }
}

bool DataPool::removeConnection(DataPool::Connection *connection) {
  for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
    if (*it == connection) {
      delete connection;
      connections.erase(it);
      return true;
    }
  }
  return false;
}

DataPool::Connections DataPool::getConnections() const {
  return connections;
}

DataPool::Ports DataPool::getInputPorts() const {
  return inputPorts.getPorts();
}

DataPool::Ports DataPool::getOutputPorts() const {
  return outputPorts.getPorts();
}

DataPool::NewDataEvent* DataPool::getNewDataEvent() {
  return &newDataEvent;
}

void DataPool::newData(DataPool::Connection* connection) {
  newDataEvent(connection);
}

DataPool::Connection* DataPool::find(const std::string& name, const RTT::types::TypeInfo *typeInfo) {
  for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
    Connection *connection = *it;
    if (connection->name == name && (!typeInfo || connection->typeInfo == typeInfo)) return connection;
  }

  return 0;
}

DataPool::Connection* DataPool::find(const Data::Key key, const RTT::types::TypeInfo *typeInfo) {
  for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
    Connection *connection = *it;
    if (connection->key == key && (!typeInfo || connection->typeInfo == typeInfo)) return connection;
  }

  return 0;
}

RTT::base::OutputPortInterface *DataPool::getOutputPort(const std::string& name, const RTT::types::TypeInfo *typeInfo) {
  Connection *connection = find(name, typeInfo);
  if (!connection) return 0;
  return connection->outputPort;
}

RTT::base::OutputPortInterface *DataPool::getOutputPort(const Data::Key key, const RTT::types::TypeInfo *typeInfo) {
  Connection *connection = find(key, typeInfo);
  if (!connection) return 0;
  return connection->outputPort;
}

RTT::base::InputPortInterface *DataPool::getInputPort(const std::string& name, const RTT::types::TypeInfo *typeInfo) {
  Connection *connection = find(name, typeInfo);
  if (!connection) return 0;
  return connection->inputPort;
}

RTT::base::InputPortInterface *DataPool::getInputPort(const Data::Key key, const RTT::types::TypeInfo *typeInfo) {
  Connection *connection = find(key, typeInfo);
  if (!connection) return 0;
  return connection->inputPort;
}

} // namespace uxvcos
