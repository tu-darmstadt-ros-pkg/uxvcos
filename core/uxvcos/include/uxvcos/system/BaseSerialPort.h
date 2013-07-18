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

#ifndef SYSTEM_BASESERIALPORT_H
#define SYSTEM_BASESERIALPORT_H

#include <string>
#include <stdexcept>

#include <uxvcos/stream/Stream.h>
#include <uxvcos/Time.h>

#include <uxvcos/uxvcos.h>

template <typename T> class Buffer;

namespace System {

class UXVCOS_API BaseSerialPort : public Stream {
public:
  BaseSerialPort() {}
  virtual ~BaseSerialPort() {}

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool blocking(bool) = 0;
  virtual bool isOpen() const = 0;

  virtual int send(const void *source, size_t size) = 0;
  virtual int receive(void *source, size_t size) = 0;
  virtual int bytesAvailable() const = 0;

  virtual bool wait(unsigned long msecs) = 0;
  virtual void flush();

  virtual bool setDevice(std::string device) = 0;
  virtual bool setBaudrate(unsigned long b) = 0;
  virtual uxvcos::Time getTimestamp() const { return uxvcos::Time(); }

  virtual bool setRequestToSend(bool enable) const     { return false; }
  virtual bool getClearToSend() const                  { return false; }
  virtual bool setDataTerminalReady(bool enable) const { return false; }
  virtual bool getDataSetReady() const                 { return false; }
  virtual bool getCarrierDetect() const                { return false; }
  virtual bool getRingIndicator() const                { return false; }

  // InStream members
  virtual element_t get() {
    element_t c;
    if (receive(&c, sizeof(c)) == sizeof(c)) return c;
    return EoF;
  }
  virtual size_t readsome(void *destination, size_t n) {
    int ret = receive(destination, n);
    _eof = (ret < 0);
    if (ret < 0) return 0;
    return ret;
  }

//  virtual size_t size() const {
//    return bytesAvailable();
//  }

  // OutStream members
  virtual OutStream& write(const void *source, size_t size) {
    int ret = send(source, size);
    _overflow = (ret < 0 || (size_t) ret != size);
    return *this;
  };

  // Stream members
  virtual bool good() const {
    return isOpen() && Stream::good();
  }

  virtual void reset() {
    BaseStream::reset();
    readBuffer.reset();
    writeBuffer.reset();
  }

  class UXVCOS_API Exception : public std::exception {
  public:
    Exception(const std::string& reason) throw() :
      reason(reason) {}
    virtual ~Exception() throw() {}
    virtual const char* what() const throw() { return reason.c_str(); }
  private:
    std::string reason;
  };

  // ROS compatibility methods
  virtual element_t *data();
  virtual element_t *del(ssize_t len);
  virtual element_t *tail();
  virtual element_t *add(ssize_t len);

private:
  boost::shared_ptr<Buffer<uint8_t> > readBuffer;
  boost::shared_ptr<Buffer<uint8_t> > writeBuffer;
};

UXVCOS_API BaseSerialPort *getSerialPort(const std::string& device);
UXVCOS_API void registerSerialPort(BaseSerialPort* port, const std::string& device);
UXVCOS_API void unregisterSerialPort(BaseSerialPort* port);

} // namespace System

#endif // SYSTEM_BASESERIALPORT_H
