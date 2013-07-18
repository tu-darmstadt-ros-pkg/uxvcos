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

#include "EthernetSerialPort.h"
#include <uxvcos/system/Socket.h>
#include <uxvcos/system/SystemCall.h>

#include <rtt/Activity.hpp>
#include <rtt/os/MutexLock.hpp>
#include <rtt/os/fosi.h>

namespace uxvcos {
namespace Interface {

  class EthernetSerialPort::Worker : public RTT::base::RunnableInterface
  {
  public:
    Worker(EthernetSerialPort *owner);
    ~Worker();

    bool start();
    void stop();

    bool initialize();
    void loop();
    void step();
    bool breakLoop();
    void finalize();

    bool send(const void *source, size_t size);

  private:
    EthernetSerialPort *owner;
    RTT::Activity activity;
    bool is_running;

    RTT::os::Mutex mutex;

    System::BaseSocket *socket;
    System::BaseSocket *client;
    char buffer[1024];
  };

  EthernetSerialPort::EthernetSerialPort(RTT::TaskContext* parent, int index, const std::string& name, const std::string& description)
    : Module(parent, name, description)
    , handler(dynamic_cast<Handler*>(parent))
    , index(index)
    , baudrate("Baudrate", "Baudrate of the serial port", 115200)
    , _isOpen(false)
    , worker(new Worker(this))
  {
    if (name.empty()) {
      std::ostringstream ss;
      ss << "Ethernet" << index;
      setName(ss.str());
    }

    RTT::Logger::In _in(getName());

    this->addOperation("open", &EthernetSerialPort::open, this).doc("Opens the serial connection");
    this->addOperation("close", &EthernetSerialPort::close, this).doc("Closes the serial connection");
    this->addOperation("isOpen", &EthernetSerialPort::isOpen, this).doc("Returns true, if the serial connection is opened");

    this->addOperation("setBaudrate", &EthernetSerialPort::setBaudrate, this).doc("Sets the baudrate of the serial port.").arg("baudrate", "Real baudrate is hardware-dependent!");
    this->addOperation("sendString", &EthernetSerialPort::sendString, this).doc("Sends a string to the serial port.").arg("string", "String to be sent");
    this->addOperation("receiveString", &EthernetSerialPort::receiveString, this).doc("Sends a string to the serial port.").arg("string", "String to be sent");

    this->addProperty(baudrate);

    System::registerSerialPort(this, getName());
    RTT::log(RTT::Debug) << "Registered serial port " << getName() << RTT::endlog();
  }

  EthernetSerialPort::~EthernetSerialPort() {
    System::unregisterSerialPort(this);
    delete(worker);
  }

  bool EthernetSerialPort::initialize()
  {
    worker->start();
    return true;
  }

  void EthernetSerialPort::cleanup()
  {
    RTT::Logger::In _in(getName());
    worker->stop();
    close();
  }

  bool EthernetSerialPort::open() {
    RTT::Logger::In _in(getName());

    if (!handler) {
      RTT::log( RTT::Error ) << "No handler defined for EthernetSerialPort " << getName() << RTT::endlog();
      return false;
    }
    if (!handler->setBaudrate(this, baudrate)) {
      return false;
    }

    if (!_isOpen) {
      _isOpen = true;
      RTT::log(RTT::Info) << "Opened serial port Ethernet" << index << RTT::endlog();
    }
    return true;
  }

  void EthernetSerialPort::close() {
    RTT::Logger::In _in(getName());
    if (_isOpen) {
      _isOpen = false;
      RTT::log(RTT::Info) << "Closed serial port Ethernet" << index << RTT::endlog();
    }
  }

  bool EthernetSerialPort::isOpen() const {
    return _isOpen;
  }

  bool EthernetSerialPort::blocking(bool b) { return false; }

  int EthernetSerialPort::send(const void *source, size_t size) {
    RTT::Logger::In _in(getName());

    if (!isOpen() && !open()) return -1;
    if (!handler) {
      RTT::log( RTT::Error ) << "No handler defined for EthernetSerialPort " << getName() << RTT::endlog();
      return -1;
    }
    if (!handler->uartSend(this, source, size)) return 0;
    return size;
  }

  bool EthernetSerialPort::setDevice(std::string device) {
    return true;
  }

  bool EthernetSerialPort::setBaudrate(unsigned long b) {
    RTT::Logger::In _in(getName());

    baudrate.set(b);
    if (!handler) {
      RTT::log( RTT::Error ) << "No handler defined for EthernetSerialPort " << getName() << RTT::endlog();
      return false;
    }
    if (!_isOpen) return true;
    return handler->setBaudrate(this, b);
  }

  int EthernetSerialPort::receive(void *source, size_t size) {
    RTT::Logger::In _in(getName());
    if (in.size() == 0 && !poll()) return 0;
    return in.readsome(source, size);
  }

  uxvcos::Time EthernetSerialPort::getTimestamp() const {
    return timestamp;
  }

  int EthernetSerialPort::bytesAvailable() const {
    return in.size();
  }

  bool EthernetSerialPort::wait(unsigned long msecs) {
    RTT::os::TimeService::ticks start = RTT::os::TimeService::Instance()->getTicks();
    while(in.size() == 0 && RTT::os::TimeService::Instance()->secondsSince(start) * 1e3 < msecs) {
      if (!poll()) return false;
      struct timespec timeout = { 0, 10000000 }; // 10ms
      rtos_nanosleep(&timeout, 0);
    }
    return (in.size() > 0);
  }

  bool EthernetSerialPort::poll() {
    if (!handler) {
      RTT::log( RTT::Error ) << "No handler defined for EthernetSerialPort " << getName() << RTT::endlog();
      return false;
    }
    handler->uartPoll(this);
    return true;
  }

  bool EthernetSerialPort::receiveHandler(void* source, unsigned int length, const uxvcos::Time timestamp) {
    RTT::Logger::In _in(getName());

    if (!_isOpen) return false;
    worker->send(source, length);

    this->timestamp = timestamp;
    return in.write(source, length).good();
  }

  int EthernetSerialPort::sendString(std::string str) {
    return send(str.data(), str.length());
  }

  std::string EthernetSerialPort::receiveString() {
    const size_t size = 1024;
    char buf[size];
    if (receive(buf, size)) {
      return std::string(buf, sizeof(buf) - size);
    }
    return std::string();
  }

  EthernetSerialPort::Worker::Worker(EthernetSerialPort *owner)
    : owner(owner)
    , activity(this, owner->getName() + "Activity")
    , is_running(false)
    , socket(0)
    , client(0)
  {}

  EthernetSerialPort::Worker::~Worker() {
    stop();
  }

  bool EthernetSerialPort::Worker::start() { return activity.start(); }
  void EthernetSerialPort::Worker::stop() { activity.stop(); }

  bool EthernetSerialPort::Worker::initialize()
  {
    socket = new System::Socket(System::BaseSocket::SOCKET_TCP);
    if (!socket->create() || !socket->setNonBlocking(true) || !socket->setReuseAddr(true) || !socket->bind(System::NetworkAddress::any(60000 + owner->getIndex())) || !socket->listen()) {
      finalize();
      return false;
    }

    is_running = true;
    return true;
  }

  void EthernetSerialPort::Worker::finalize()
  {
    if (client) {
      client->close();
      delete client;
      client = 0;
    }

    if (socket) {
      socket->close();
      delete socket;
      socket = 0;
    }

    is_running = false;
  }

  void EthernetSerialPort::Worker::loop() {
    while(is_running && socket->isValid()) step();
  }

  void EthernetSerialPort::Worker::step() {
    RTT::Logger::In _in(owner->getName());
    static const unsigned long timeout = 100;

    if (!client) {
      // RTT::log(RTT::Info) << "Waiting for incoming connection..." << RTT::endlog();
      if (!socket->wait(timeout)) return;

      RTT::os::MutexLock lock(mutex);
      System::NetworkAddress clientAddress;
      client = socket->accept(clientAddress);
      if (client) {
        // client->setNonBlocking(false);
        RTT::log(RTT::Info) << "Connected socket " << static_cast<int>(*client) << " to " << clientAddress.toString() << RTT::endlog();
        if (!owner->isOpen()) owner->open();
      }
    }

    if (client) {
      if (!client->isConnected()) {
        RTT::os::MutexLock lock(mutex);
        RTT::log(RTT::Info) << "Client is not connected." << RTT::endlog();
        delete client; client = 0;
        return;
      }

      // RTT::log(RTT::Info) << "Waiting for data from client..." << RTT::endlog();
      if (!client->wait(timeout)) return;
      size_t length = client->readsome(buffer, sizeof(buffer));
      if (length > 0) {
        RTT::log(RTT::Debug) << "Sending message with " << length << " bytes from external client" << RTT::endlog();
        owner->send(buffer, length);
      }

      if (!client->isConnected()) {
        RTT::os::MutexLock lock(mutex);
        RTT::log(RTT::Info) << "Disconnected." << RTT::endlog();
        delete client; client = 0;
        return;
      }
    }
  }

  bool EthernetSerialPort::Worker::send(const void *source, size_t size) {
    RTT::os::MutexLock lock(mutex);
    if (!client) return false;
    RTT::Logger::In _in(owner->getName());
    RTT::log(RTT::Debug) << "Sending message with " << size << " bytes to the external client" << RTT::endlog();
    return client->send(source, size);
  }

  bool EthernetSerialPort::Worker::breakLoop() {
    is_running = false;
    return true;
  }

} // namespace Interface
} // namespace uxvcos
