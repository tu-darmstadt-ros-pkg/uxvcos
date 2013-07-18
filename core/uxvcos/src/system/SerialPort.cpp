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

#include "system/BaseSerialPort.h"
#include "system/SerialPort.h"
#include "stream/Buffer.h"
#include "system/NameServer.h"

namespace {
  System::NameServer<System::BaseSerialPort *> nameServer;
}

namespace System {
  BaseSerialPort *getSerialPort(const std::string& device) {
    if (device.empty()) return 0;
    BaseSerialPort *serialPort = 0;

    serialPort = nameServer.getObject(device);

    if (!serialPort) serialPort = new SerialPort(device);
    return serialPort;
  }

  void registerSerialPort(BaseSerialPort* port, const std::string& device) {
    nameServer.registerObject(port, device);
  }

  void unregisterSerialPort(BaseSerialPort* port) {
    nameServer.unregisterObject(port);
  }

  void BaseSerialPort::flush() {
    if (writeBuffer) {
      send(writeBuffer->data(), writeBuffer->size());
      writeBuffer->clear();
    }
  }

  BaseStream::element_t *BaseSerialPort::data() {
    if (!readBuffer) readBuffer.reset(new Buffer<uint8_t>());
    return readBuffer->data();
  }

  BaseStream::element_t *BaseSerialPort::del(BaseStream::ssize_t len) {
    if (len < 0) return 0;
    if (!readBuffer) readBuffer.reset(new Buffer<uint8_t>());

    if (readBuffer->size() < (size_t) len) {
      size_t bytes_to_read = len - readBuffer->size();
      element_t *destination = readBuffer->add(bytes_to_read);
      if (!destination) return 0;
      if (receive(destination, bytes_to_read) != static_cast<int>(bytes_to_read)) return 0;
    }
    return readBuffer->del(len);
  }

  BaseStream::element_t *BaseSerialPort::tail() {
    if (!writeBuffer) writeBuffer.reset(new Buffer<uint8_t>());
    return writeBuffer->tail();
  }

  BaseStream::element_t *BaseSerialPort::add(BaseStream::ssize_t len) {
    if (len < 0) return 0;
    if (!writeBuffer) writeBuffer.reset(new Buffer<uint8_t>());
    return writeBuffer->add(len);
  }
}
