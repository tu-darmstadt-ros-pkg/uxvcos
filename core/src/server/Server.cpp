#include "Server.h"

#include <system/Socket.h>
#include <system/Error.h>

#include <rtt/Activity.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>

#include <options/options.h>
#include <base/SetupFunction.h>
#include <base/Application.h>

namespace uxvcos {

static const int priority = 0;

namespace { SetupFunction setup(&Server::setup, "Server"); }
  
Server::Server(const std::string& name, DataPool *dataPool, int capacity)
  : RTT::TaskContext(name, PreOperational)
  , type("Type", "Server type (TCP,UDP)", "UDP")
  , destinationHost("DestinationAddress", "Destination address (for UDP only)", "127.0.0.1")
  , destinationPort("DestinationPort", "Destination port (for UDP only)", 32255)
  , localPort("LocalPort", "Local port", 32255)
  , socket(0)
  , acceptor(0)
  , sendBuffer(65536)
  , encoder(&sendBuffer)
  , dataPool(dataPool ? dataPool : DataPool::Global())
  , portListener(this, false, RTT::ConnPolicy::buffer(100))
{
  portListener.useDataPool(dataPool);

  this->addProperty(type);
  this->addProperty(destinationHost);
  this->addProperty(destinationPort);
  this->addProperty(localPort);

  this->addOperation("addConnection", &Server::addConnection, this, RTT::OwnThread);
  this->addOperation("removeConnection", &Server::removeConnection, this, RTT::OwnThread);

  connections.reserve(capacity);

  this->setActivity(new RTT::Activity(priority, 0, this->getName()));
}

Server::~Server()
{
  stop();
  cleanup();
}

int Server::setup() {
  Options::Description options("Server options");
  options.add_options()
    ("destination,d", Options::value<std::string>(), "destination IP address");
  options.add_options()
    ("rate", Options::value<int>(), "packets per second (default is one packet per main step)");
  Options::options().add(options);
  return 0;
}

bool Server::configureHook()
{
  RTT::Logger::In in(getName());

  if (!Options::variables("destination").empty()) destinationHost.set(Options::variables("destination").as<std::string>());
  if (!Options::variables("rate").empty()) {
    int rate = Options::variables("rate").as<int>();
    if (rate == 0) this->setPeriod(0.0); else this->setPeriod(1.0/rate);
  }

  return true;
}

bool Server::startHook()
{
  RTT::Logger::In in(getName());

  if (socket) {
    RTT::log( RTT::Error ) << "Socket already exists!" << RTT::endlog();
    return false;
  }

  System::Socket::SocketType socketType;
  if (type.get() == "TCP")
    socketType = System::Socket::SOCKET_TCP;
  else if (type.get() == "UDP")
    socketType = System::Socket::SOCKET_UDP;
  else {
    RTT::log( RTT::Error ) << "Type must be set to either TCP or UDP" << RTT::endlog();
    return false;
  }

  socket = new System::Socket(socketType);

  if (!socket->create()) {
    RTT::log( RTT::Error ) << "Could not create socket: " << System::lastError() << RTT::endlog();
    stopHook();
    return false;
  }

  socket->setBroadcast(true);
  socket->setReuseAddr(true);
  socket->setNonBlocking(true);

  if (!socket->bindAny(static_cast<unsigned short>(localPort.get()))) {
    RTT::log( RTT::Error ) << "Could not bind socket to port " << localPort.get() << ": " << System::lastError() << RTT::endlog();
    stopHook();
    return false;
  }

  if (type.get() == "TCP") {
    if (!socket->listen()) {
      RTT::log( RTT::Error ) << "Could not listen on TCP port " << localPort.get() << ": " << System::lastError() << RTT::endlog();
      stopHook();
      return false;
    }

    acceptor = new SocketAcceptor(this, *socket);
    if (!acceptor->start()) {
      RTT::log( RTT::Error ) << "Could not start socket acceptor" << RTT::endlog();
      stopHook();
      return false;
    }

  } else {
    System::NetworkAddress destination;
    
    if (!destination.set(destinationHost.get(), static_cast<unsigned short>(destinationPort.get())) || !socket->connect(destination)) {
      RTT::log( RTT::Error ) << "Could not connect to destination " << destinationHost.get() << ":" << destinationPort.get() << ": " << System::lastError() << RTT::endlog();
      stopHook();
      return false;
    }

    // put socket in the list of client connections
    addConnection(new ClientConnection(this, *socket));
  }

  // listen to all available ports
  portListener.start();
  
  return true;
}

void Server::stopHook()
{
  RTT::Logger::In in(getName());

  // stop listening
  portListener.stop();

  // stop socket acceptor (for TCP sockets)
  if (acceptor) {
    delete acceptor;
    acceptor = 0;
  }
  
  // kill all client connections
  connections.clear();
  
  if (socket) {
    if (type.get() == "TCP") socket->close();
    delete socket;
    socket = 0;
  }
}

void Server::cleanupHook()
{
  RTT::Logger::In in(getName());
}

bool Server::send(RTT::base::DataSourceBase::shared_ptr dsb) {
  Buffer<>::MutexLock lock(&sendBuffer);

  if (!(encoder << dsb)) {
    RTT::log(RTT::RealTime) << "Failed to send DataSource of type " << dsb->getType() << RTT::endlog();
    return false;
  }
  return true;
}

bool Server::send(const Data::Streamable& message) {
  Buffer<>::MutexLock lock(&sendBuffer);

  if (!(encoder << message)) {
    RTT::log(RTT::RealTime) << "Failed to send object of type ";
    if (message.getTypeInfo())
      RTT::log() << message.getTypeInfo()->getTypeName();
    else
      RTT::log() << "(unknown: " << typeid(message).name() << ")";
    RTT::log() << RTT::endlog();
    return false;
  }
  return true;
}

void Server::updateHook()
{
  // RTT::Logger::In in(getName());
  Buffer<>::MutexLock lock(&sendBuffer);

  const PortListener::InputPorts &ports = portListener.getPorts();
  for(PortListener::InputPorts::const_iterator it = ports.begin(); it != ports.end(); it++) {
    RTT::base::DataSourceBase::shared_ptr dsb = (*it)->getDataSource();
    while(dsb->evaluate()) send(dsb);
  }
  // portListener.free();

  if (sendBuffer.size() > 0) {
    for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
      ClientConnection::shared_ptr connection = *it;
      connection->send(sendBuffer.data(), sendBuffer.size());
    }

    sendBuffer.clear();
  }

  for(Connections::iterator it = connections.begin(); it != connections.end(); ) {
    ClientConnection::shared_ptr connection = *it;
    if (!connection->alive()) {
      it = removeConnectionImpl(it);
      continue;
    }
    ++it;
  }
}

bool Server::breakUpdateHook() {
  return true;
}

bool Server::ClientConnection::initialize() {
  if (!socket.isValid()) return false;
  RTT::log(RTT::Info) << "Connection from " << socket.getRemoteAddress().toString() << RTT::endlog();
  return true;
}

void Server::ClientConnection::loop() {
  Data::Streamable *data;
  bool trigger = false;

  if (!alive()) return;

  // read new packet socket
  if (socket.wait(100)) {
    socket >> in;

    while((data = decoder->read())) {
      if (data == Data::Streamable::BREAK) continue;

      server->data()->inject(data);
      trigger = true;
    }
  }

  // trigger main application if some data has been received
  if (trigger) {
    // RTT::log(RTT::Debug) << "Server received a packet, triggering main application..." << RTT::endlog();
    Application::Instance()->trigger();
  }
  this->trigger();
}

bool Server::ClientConnection::breakLoop() {
  return true;
}

void Server::ClientConnection::finalize() {
  socket.close();
  RTT::log(RTT::Info) << "Connection to " << socket.getRemoteAddress().toString() << " closed" << RTT::endlog();
}

void Server::addConnection(Server::ClientConnection *connection) {
  if (connections.size() == connections.capacity()) {
    RTT::log(RTT::Warning) << "Connection rejected because capacity limit is reached" << RTT::endlog();
    delete connection;
    return;
  }
  connections.push_back(ClientConnection::shared_ptr(connection));
  connection->start();
}

void Server::removeConnection(Server::ClientConnection *connection) {
  for(Connections::iterator it = connections.begin(); it != connections.end(); ++it) {
    if (it->get() == connection) {
      removeConnectionImpl(it);
      return;
    }
  }
}

Server::Connections::iterator Server::removeConnectionImpl(Server::Connections::iterator it) {
  return connections.erase(it);
}

bool Server::SocketAcceptor::initialize() {
  if (!socket.isValid()) return false;
  return true;
}

void Server::SocketAcceptor::loop() {
  acceptSocket = -1;

  if (socket.wait(100)) {
    // accept call
    if (socket.accept(acceptSocket, address)) {
      addConnection(new ClientConnection(server, acceptSocket));
    }
  }

  this->trigger();
}

bool Server::SocketAcceptor::breakLoop() {
  return true;
}

void Server::SocketAcceptor::finalize() {
}

ORO_LIST_COMPONENT_TYPE(Server)

} // namespace uxvcos

ORO_CREATE_COMPONENT_LIBRARY()
