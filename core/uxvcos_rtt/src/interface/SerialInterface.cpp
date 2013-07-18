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

#include "interface/SerialInterface.h"

#include <uxvcos/system/SerialPort.h>
#include <uxvcos/system/Error.h>

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
