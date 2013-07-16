#ifndef SYSTEM_BASESOCKET_H
#define SYSTEM_BASESOCKET_H

#include <string>
#include <cstring>
#include <cstdio>

#include <list>

#include <stream/Stream.h>

#include "NetworkAddress.h"
#include "EthernetAddress.h"
#include "Error.h"

#include <uxvcos.h>

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
