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

#ifndef SYSTEM_BASESOCKET_H
#define SYSTEM_BASESOCKET_H

#include <string>
#include <cstring>
#include <cstdio>

#include <list>

#include <uxvcos/stream/Stream.h>

#include "NetworkAddress.h"
#include "EthernetAddress.h"
#include "Error.h"

#include <uxvcos/uxvcos.h>

namespace System {
  
class UXVCOS_API BaseSocket : public Stream {
public:
  enum SocketType { SOCKET_UDP, SOCKET_TCP, SOCKET_PACKET };
  using BaseStream::size_t;
  using BaseStream::ssize_t;

  BaseSocket(SocketType type)
    : socketId(-1)
    , socketType(type)
  {
    if (type == SOCKET_TCP) {
      _eof = true;
      _overflow = true;
    } else {
      _eof = false;
      _overflow = false;
    }
  }

  BaseSocket(int socket, SocketType type)
    : socketId(socket)
    , socketType(type)
  {
    if (socket < 0) {
      _eof = true;
      _overflow = true;
    } else {
      _eof = false;
      _overflow = false;
    }
  }

  virtual ~BaseSocket() {}
  
  operator int&() { return socketId; }
  operator const int&() const { return socketId; }
  
  virtual BaseSocket& operator=(int socketId) {
    this->socketId = socketId;
    return *this;
  }

  virtual BaseSocket& operator=(BaseSocket const& other) {
    this->socketType = other.socketType;
    this->socketId = other.socketId;
    return *this;
  }

  virtual bool operator==(BaseSocket const& other) const {
    return (socketType == other.socketType && socketId == other.socketId);
  }

  virtual bool isValid() const { return socketId >= 0; }
  virtual bool isConnected() const { return isValid() && Stream::good(); }
  virtual bool good() const { return isConnected(); }

  SocketType getSocketType() const { return socketType; }

  virtual bool create(unsigned short protocol = 0) = 0;
  virtual bool bind(const NetworkAddress& addr) = 0;
  virtual bool bind(const std::string device, unsigned short protocol = 0, EthernetAddress* address = 0) = 0;
  virtual bool bindAny(unsigned short port) = 0;
  virtual bool listen() = 0;
  virtual bool accept(BaseSocket& acceptSocket, NetworkAddress& address) = 0;
  virtual BaseSocket *accept(NetworkAddress& addr) = 0;
  virtual bool connect(const NetworkAddress& addr, bool nonBlocking = false) = 0;
  virtual bool connect(const EthernetAddress& addr) = 0;
  virtual void close() = 0;

  virtual ssize_t receive(void* data, size_t size, bool waitall = false, bool peek = false) = 0;
  virtual bool send(const void* data, size_t size) = 0;
  virtual bool setBroadcast(bool enable) = 0;
  virtual bool setReuseAddr(bool enable) = 0;
  virtual bool setNonBlocking(bool enable) = 0;
  virtual ssize_t receiveFrom(void* data, size_t size, NetworkAddress& from, bool waitall = false, bool peek = false) = 0;
  virtual ssize_t receiveFrom(void* data, size_t size, EthernetAddress& from, bool waitall = false, bool peek = false) = 0;
  virtual bool sendTo(const void* data, size_t size, const NetworkAddress& to) = 0;
  virtual bool sendTo(const void* data, size_t size, const EthernetAddress& to) = 0;
  
  virtual bool wait(unsigned long msecs) = 0;
  virtual void flush() {}
  
  virtual OutStream& write(const void *source, size_t size) {
    send(source, size);
    return *this;
  }
  
  virtual size_t readsome(void *destination, size_t n) {
    ssize_t received = receive(destination, n);
    if (received < 0) return 0;
    return (size_t) received;
  }
  
  virtual const NetworkAddress& getRemoteAddress() const { return remoteNetworkAddress; }
  virtual void setRemoteAddress(const NetworkAddress& remote) { remoteNetworkAddress = remote; }

protected:
  int socketId;
  SocketType socketType;

  NetworkAddress remoteNetworkAddress;
};

} // namespace System

#endif // SYSTEM_BASESOCKET_H
