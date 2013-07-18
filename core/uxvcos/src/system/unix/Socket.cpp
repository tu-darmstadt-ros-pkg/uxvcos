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

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netpacket/packet.h>
#include <netinet/ether.h>
#include <netdb.h>

#include <vector>

#include "system/Socket.h"
#include "system/SystemCall.h"

namespace System {
namespace Unix {

Socket::Socket(SocketType type)
  : BaseSocket(type)
{}

Socket::Socket(int socket, SocketType type)
  : BaseSocket(socket, type)
{}

Socket::~Socket() {
}

bool Socket::create(unsigned short protocol)
{
  if (socketType == SOCKET_PACKET) {
    socketId = SystemCall::socket(AF_PACKET, SOCK_DGRAM, htons(protocol));

  } else {
    socketId = SystemCall::socket(AF_INET, (socketType == SOCKET_TCP ? SOCK_STREAM : SOCK_DGRAM), 0);

    // if the last instance of our programm crashed and the port is in state "TIME_WAIT" we want to be able to hijack the port
    // we could check the return value, but this operation is not critical
    int enable = 1;
    SystemCall::setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
  }

  return socketId != -1;
}

bool Socket::bind(const NetworkAddress& addr)
{
  if (socketId < 0) return false;
  sockaddr_in local;

  if (!addr.valid) return false;

  //memset(&local,0,sizeof(sockaddr_in)); // zuerst alles auf //0 setzten
  local.sin_addr.s_addr = addr.ip;
  local.sin_port = addr.port;
  local.sin_family = AF_INET;

  int rc = SystemCall::bind(socketId, (struct sockaddr *) (&local), sizeof(local));
  if (rc != 0) return false;

  if (socketType == SOCKET_UDP) _eof = false;
  return true;
}

bool Socket::bind(const EthernetAddress& addr)
{
  if (socketId < 0) return false;
  struct sockaddr_ll sa;

  sa.sll_family = AF_PACKET;
  sa.sll_protocol = addr.protocol;
  sa.sll_ifindex = addr.ifindex;
  sa.sll_halen = sizeof(addr.addr);
  std::memcpy(sa.sll_addr, addr.addr, sa.sll_halen);

  int rc = SystemCall::bind(socketId, (struct sockaddr *) &sa, sizeof(sa));
  if (rc != 0) return false;

  _eof = false;
  return true;
}

bool Socket::bind(const std::string interface, unsigned short protocol, EthernetAddress* addr) {
  if (socketId < 0) return false;
  EthernetAddress local;

  if (socketType != SOCKET_PACKET) return false;
  if (protocol != 0) local.protocol = htons(protocol);

  // retrieve interface information via ioctl
  struct ifreq ifr;
  strcpy(ifr.ifr_name, interface.c_str());
  if (SystemCall::ioctl(socketId, SIOCGIFHWADDR , &ifr) < 0) return false;
  std::memcpy(&local.addr, &ifr.ifr_addr.sa_data, sizeof(local.addr));

  if (SystemCall::ioctl(socketId, SIOCGIFINDEX , &ifr) < 0) return false;
  local.ifindex = ifr.ifr_ifindex;
  local.valid     = true;

  if (addr) *addr = local;
  return bind(local);
}

bool Socket::bindAny(unsigned short port)
{
  return bind(NetworkAddress::any(port));
}

bool Socket::listen()
{
  static const unsigned int SOCKET_LISTEN_BACKLOG = SOMAXCONN;
  if (socketId < 0) return false;
  int rc = SystemCall::listen(socketId, SOCKET_LISTEN_BACKLOG);
  return rc == 0;
}

Socket::SocketList::iterator Socket::accept(SocketList& socketList, NetworkAddress& addr)
{
  if (socketId < 0) return socketList.end();
  SocketList::iterator it = socketList.insert(socketList.end(), Socket(socketType));
  if (!accept(*it, addr)) {
    socketList.erase(it);
    return socketList.end();
  }
  return it;
}

bool Socket::accept(BaseSocket& acceptSocket, NetworkAddress& addr)
{
  if (socketId < 0) return false;
  sockaddr_in sa;

  error(0);
  int size = sizeof(sa);
  int id = SystemCall::accept(socketId, reinterpret_cast<sockaddr*>(&sa), reinterpret_cast<socklen_t*>(&size));
  if (id == -1) {
    if (errno == EWOULDBLOCK) return false;
    error(errno);
    // RTT::log(RTT::Debug) << std::string(__func__) << " returned error " << System::Error::lastError() << RTT::endlog();
    if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
    return false;
  }

  // set socket to nonBlocking mode
  int flags = fcntl(id, F_GETFL, 0);
  fcntl(id, F_SETFL, flags | O_NONBLOCK);

  acceptSocket = Socket(id, SOCKET_TCP);
  acceptSocket.reset();

  addr.ip   = sa.sin_addr.s_addr;
  addr.port = sa.sin_port;
  addr.valid = true;

  acceptSocket.setRemoteAddress(addr);
  return true;
}

BaseSocket *Socket::accept(NetworkAddress& addr)
{
  if (socketId < 0) return false;
  Socket acceptSocket;
  if (accept(acceptSocket, addr)) return new Socket(acceptSocket);
  return 0;
}

bool Socket::connect(const NetworkAddress& addr, bool nonBlocking)
{
  if (socketId < 0) return false;
  sockaddr_in sa;

  if (!addr.valid) return false;

  //memset(&sa,0,sizeof(sockaddr_in)); // zuerst alles auf //0 setzten
  sa.sin_addr.s_addr = addr.ip;
  sa.sin_port = addr.port;
  sa.sin_family = AF_INET;

  int flags = fcntl(socketId, F_GETFL, 0);
  if (nonBlocking) {
    fcntl(socketId, F_SETFL, flags | O_NONBLOCK);
  }

  int rc = SystemCall::connect(socketId, reinterpret_cast<sockaddr*>(&sa), sizeof(sa));
  if (rc != 0) return false;

  reset();

//     if (nonBlocking) {
//       fcntl(socketId, F_SETFL, flags & (~O_NONBLOCK));
//     }

  // set socket to nonBlocking mode after the connection has been established
  fcntl(socketId, F_SETFL, flags | O_NONBLOCK);

  remoteNetworkAddress = addr;
  return true;
}

bool Socket::connect(const EthernetAddress& addr)
{
  if (socketId < 0) return false;
  remoteEthernetAddress = addr;
  reset();
  return true;
}

void Socket::close()
{
  if (socketId < 0) return;
  SystemCall::close(socketId);
  _eof = true;
  _overflow = true;
  socketId = -1;
}

bool Socket::setBroadcast(bool enable)
{
  if (socketId < 0) return false;
  int value = enable ? 1 : 0;

  error(0);
  if (setsockopt(socketId, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&value), sizeof(value)) == -1) {
    error(errno);
    return false;
  }

  return true;
}

bool Socket::setReuseAddr(bool enable)
{
  if (socketId < 0) return false;
  int value = enable ? 1 : 0;

  error(0);
  if (setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&value), sizeof(value)) == -1) {
    error(errno);
    return false;
  }

  return true;
}

bool Socket::setNonBlocking(bool enable)
{
  if (socketId < 0) return false;
  int flags;

  if (-1 == (flags = fcntl(socketId, F_GETFL, 0)))
      flags = 0;
  if (enable)
    flags |= O_NONBLOCK;
  else
    flags &= ~O_NONBLOCK;

  return (0 == fcntl(socketId, F_SETFL, flags));
}

BaseStream::ssize_t Socket::receive(void* data, BaseStream::size_t size, bool waitall, bool peek)
{
  if (socketId < 0) return false;

  if (socketType == SOCKET_PACKET) {
    EthernetAddress sender = remoteEthernetAddress;
    return receiveFrom(data, size, sender);
  }

  int flags = 0;
  if (peek)    flags |= MSG_PEEK;
  if (waitall) flags |= MSG_WAITALL;

  error(0);
  ssize_t bytesRecv = SystemCall::recv(socketId, data, size, flags);

  if (bytesRecv == -1) {
    if (errno == EWOULDBLOCK) {
      bytesRecv = 0;
    } else {
      error(errno);
      if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
    }
  }

  return bytesRecv;
}

BaseStream::ssize_t Socket::receiveFrom(void* data, BaseStream::size_t size, NetworkAddress& from, bool waitall, bool peek)
{
  if (socketId < 0) return false;
  sockaddr_in src;
  socklen_t srclen = sizeof(src);
  memset(&src, 0, srclen);

  int flags = 0;
  if (peek) {
    flags |= MSG_PEEK;
  }
  if (waitall) {
    flags |= MSG_WAITALL;
  }

  error(0);
  ssize_t bytesRecv = SystemCall::recvfrom(socketId, static_cast<char*>(data), size, flags, reinterpret_cast<sockaddr*>(&src), &srclen);

  if (bytesRecv == -1) {
    if (errno == EWOULDBLOCK) {
      bytesRecv = 0;
    } else {
      error(errno);
      if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
    }
  }

  from.ip = src.sin_addr.s_addr;
  from.port = src.sin_port;

  return bytesRecv;
}

BaseStream::ssize_t Socket::receiveFrom(void* data, BaseStream::size_t size, EthernetAddress& from, bool waitall, bool peek)
{
  if (socketId < 0) return -1;
  if (socketType != SOCKET_PACKET) return -1;

  struct sockaddr_ll src;
  socklen_t srclen = sizeof(src);
  memset(&src, 0, srclen);

  int flags = 0;
  if (peek) {
    flags |= MSG_PEEK;
  }
  if (waitall) {
    flags |= MSG_WAITALL;
  }

  error(0);
  ssize_t bytesRecv = SystemCall::recvfrom(socketId, static_cast<char*>(data), size, flags, reinterpret_cast<sockaddr*>(&src), &srclen);

  if (bytesRecv == -1) {
    if (errno == EWOULDBLOCK) {
      bytesRecv = 0;
    } else {
      error(errno);
      if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
    }
  }

  from.protocol = src.sll_protocol;
  from.ifindex = src.sll_ifindex;
  memcpy(from.addr, src.sll_addr, sizeof(from.addr));

  return bytesRecv;
}

bool Socket::send(const void* data, size_t size)
{
  if (socketId < 0) return false;
  if (socketType == SOCKET_PACKET) return sendTo(data, size, remoteEthernetAddress);

  error(0);
  int bytesSend = static_cast<int>(SystemCall::send(socketId, data, size, 0));
  if (bytesSend == -1) {
    error(errno);
    // RTT::log(RTT::Debug) << std::string(__func__) << " returned error " << System::Error::lastError() << RTT::endlog();
    if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
  } else if (bytesSend != (long)size) {
    return false;
  }

  return true;
}

bool Socket::sendTo(const void* data, size_t size, const NetworkAddress& to)
{
  if (socketId < 0) return false;
  sockaddr_in sa;

  if (!to.valid) return false;

  //memset(&sa,0,sizeof(sockaddr_in)); // zuerst alles auf //0 setzten
  sa.sin_addr.s_addr = static_cast<in_addr_t>(to.ip);
  sa.sin_port = to.port;
  sa.sin_family = AF_INET;

  error(0);
  int bytesSend = static_cast<int>(SystemCall::sendto(socketId, static_cast<const char*>(data), size, 0, reinterpret_cast<const sockaddr*>(&sa), sizeof(sa)));

  if (bytesSend == -1) {
    error(errno);
    // RTT::log(RTT::Debug) << std::string(__func__) << " returned error " << System::Error::lastError() << RTT::endlog();
    if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
  } else if ((size_t) bytesSend != size) {
    return false;
  }

  return true;
}

bool Socket::sendTo(const void* data, size_t size, const EthernetAddress& to)
{
  if (socketId < 0) return false;
  struct sockaddr_ll sa;

  if (socketType != SOCKET_PACKET) return false;

  sa.sll_family = AF_PACKET;
  sa.sll_protocol = to.protocol;
  sa.sll_ifindex = to.ifindex;
  sa.sll_halen = sizeof(to.addr);
  std::memcpy(sa.sll_addr, to.addr, sa.sll_halen);

  error(0);
  int bytesSend = static_cast<int>(SystemCall::sendto(socketId, static_cast<const char*>(data), size, 0, reinterpret_cast<const sockaddr*>(&sa), sizeof(sa)));

  if (bytesSend == -1) {
    error(errno);
    // RTT::log(RTT::Debug) << std::string(__func__) << " returned error " << System::Error::lastError() << RTT::endlog();
    if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
  } else if ((size_t) bytesSend != size) {
    return false;
  }

  return true;
}

bool Socket::wait(unsigned long msecs) {
  if (socketId < 0) return false;
  fd_set rfds;
  struct timeval tv;
  int ret;

  FD_ZERO(&rfds);
  FD_SET(socketId, &rfds);

  tv.tv_sec = msecs / 1000;
  tv.tv_usec = msecs * 1000;

  error(0);
  ret = SystemCall::select(socketId + 1, &rfds, NULL, NULL, &tv);
  /* Don't rely on the value of tv now! */

  if (ret == -1) {
    error(errno);
    // RTT::log(RTT::Debug) << std::string(__func__) << " returned error " << System::Error::lastError() << RTT::endlog();
    if (_error == ECONNRESET || _error == EBADF || _error == EPIPE) close();
  }

  return (ret > 0);
}

void Socket::flush() {
  char temp[256];
  size_t s = sizeof(temp);
  while(wait(0) && receive(temp, s, true) >= 0) {}
}

} // namespace Unix

bool System::NetworkAddress::set(std::string ip, uint16_t port) {
  struct hostent* host = gethostbyname(ip.c_str());
  if (!host) { valid = false; return false; }
  valid = true;
  this->ip = *(reinterpret_cast<unsigned long *>(host->h_addr_list[0]));
  if (port != 0) this->port = htons(port);
  return true;
}

} // namespace System
