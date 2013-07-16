#include "SerialInterface.h"

#include <system/SerialPort.h>
#include <system/Error.h>

#include <rtt/Component.hpp>

namespace uxvcos {
namespace Interface {

SerialInterface::SerialInterface(const std::string& name)
  : StreamInterface(name)
  , deviceName("DeviceName", "Name of the serial device (/dev/ttyX or COMx)")
  , baudRate("Baudrate", "Baudrate of the serial device")
  , serialPort(0)
{
  serialPort = new System::SerialPort();
  this->setInStream(serialPort);
  this->setOutStream(serialPort);

  this->properties()->addProperty(deviceName);
  this->properties()->addProperty(baudRate);
}

SerialInterface::~SerialInterface() {
  stop();
  cleanup();

  StreamInterface::inStream = 0;
  StreamInterface::outStream = 0;
  delete serialPort;
}

bool SerialInterface::configureHook() {
  RTT::Logger::In in( this->getName() );

  if (!serialPort->setDevice(deviceName.get())) {
    RTT::log( RTT::Error ) << "Illegal device name: " << deviceName.get() << RTT::endlog();
    return false;
  }

  if (!serialPort->setBaudrate(baudRate.get())) {
    RTT::log( RTT::Error ) << "Could not set baudrate " << baudRate.get() << " on " << deviceName.get() << RTT::endlog();
    return false;
  }

  return StreamInterface::configureHook();
}

bool SerialInterface::startHook() {
  RTT::Logger::In in( this->getName() );

  if (!serialPort->open()) {
    RTT::log( RTT::Error ) << "Could not open port " << deviceName.get() << ": " << System::lastError() << RTT::endlog();
    return false;
  }

  return StreamInterface::startHook();
}

void SerialInterface::stopHook() {
  RTT::Logger::In in( this->getName() );

  StreamInterface::stopHook();
  serialPort->close();
}

void SerialInterface::cleanupHook() {
  RTT::Logger::In in( this->getName() );

  StreamInterface::cleanupHook();
}
ORO_CREATE_COMPONENT(SerialInterface)

} // namespace Interface
} // namespace uxvcos
