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

#ifndef SYSTEM_UNIX_SOCKET_H
#define SYSTEM_UNIX_SOCKET_H

#include "../BaseSocket.h"

namespace System {
namespace Unix {

class UXVCOS_API Socket : public BaseSocket {
public:
  using BaseSocket::SocketType;
  typedef std::vector<Socket> SocketList;

  Socket(SocketType type = SOCKET_UDP);
  Socket(int socket, SocketType type);
  virtual ~Socket();

  using BaseSocket::operator=;
  using BaseSocket::operator==;

  bool create(unsigned short protocol = 0);
  bool bind(const NetworkAddress& addr);
  bool bind(const EthernetAddress& addr);
  bool bind(const std::string interface, unsigned short protocol = 0, EthernetAddress* addr = 0);
  bool bindAny(unsigned short port);
  bool listen();
  SocketList::iterator accept(SocketList& socketList, NetworkAddress& addr);
  bool accept(BaseSocket& acceptSocket, NetworkAddress& addr);
  BaseSocket *accept(NetworkAddress& addr);
  bool connect(const NetworkAddress& addr, bool nonBlocking = false);
  bool connect(const EthernetAddress& addr);
  void close();

  bool setBroadcast(bool enable);
  bool setReuseAddr(bool enable);
  bool setNonBlocking(bool enable);

  ssize_t receive(void* data, size_t size, bool waitall = false, bool peek = false);
  ssize_t receiveFrom(void* data, size_t size, NetworkAddress& from, bool waitall = false, bool peek = false);
  ssize_t receiveFrom(void* data, size_t size, EthernetAddress& from, bool waitall = false, bool peek = false);
  bool send(const void* data, size_t size);
  bool sendTo(const void* data, size_t size, const NetworkAddress& to);
  bool sendTo(const void* data, size_t size, const EthernetAddress& to);

  bool wait(unsigned long msecs);
  void flush();

  const EthernetAddress& getRemoteEthernetAddress() const { return remoteEthernetAddress; }

private:
  EthernetAddress remoteEthernetAddress;
};

} // namespace Unix
} // namespace System

#endif // SYSTEM_UNIX_SOCKET_H
