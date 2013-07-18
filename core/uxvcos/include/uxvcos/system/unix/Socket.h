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
