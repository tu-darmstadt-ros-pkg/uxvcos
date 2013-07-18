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

#ifndef INTERFACE_ETHERNETSERIALPORT_H
#define INTERFACE_ETHERNETSERIALPORT_H

#include <uxvcos/Module.h>
#include <uxvcos/system/BaseSerialPort.h>

#include <uxvcos/stream/Buffer.h>

#include <rtt/Operation.hpp>
#include <rtt/Logger.hpp>

namespace uxvcos {
namespace Interface {

class EthernetSerialPort : public Module, public System::BaseSerialPort
{
public:
  class Handler {
  public:
    virtual ~Handler() {}
    virtual bool setBaudrate(EthernetSerialPort* port, unsigned long baudrate) = 0;
    virtual int uartSend(EthernetSerialPort* port, const void* source, unsigned int length) = 0;
    virtual bool uartPoll(EthernetSerialPort* port) = 0;
  };

  EthernetSerialPort(RTT::TaskContext* parent, int index, const std::string& name = "SerialPort", const std::string& description = "Interface to the Serial Port on the Ethernet Board");
  virtual ~EthernetSerialPort();

  virtual bool open();
  virtual void close();
  virtual bool isOpen() const;

  virtual bool blocking(bool b);
  virtual int send(const void *source, size_t size);

  virtual bool setDevice(std::string device);
  virtual bool setBaudrate(unsigned long b);

  virtual int receive(void *source, size_t size);
  virtual uxvcos::Time getTimestamp() const;
  virtual int bytesAvailable() const;

  virtual bool wait(unsigned long msecs);

  virtual bool initialize();
  virtual void cleanup();

  virtual bool receiveHandler(void* source, unsigned int length, const uxvcos::Time timestamp);

  int getIndex() const { return index; }

protected:
  Buffer<unsigned char> in;
  Handler *handler;
  int index;
  RTT::Property<unsigned int> baudrate;
  bool _isOpen;
  uxvcos::Time timestamp;

  // bool setBaudrate(unsigned int b);
  int sendString(std::string str);
  std::string receiveString();

private:
  class Worker;
  Worker *worker;

  bool poll();
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETSERIALPORT_H
