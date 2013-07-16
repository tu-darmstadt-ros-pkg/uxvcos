#ifndef SYSTEM_WIN32_SOCKET_H
#define SYSTEM_WIN32_SOCKET_H

#include <winsock2.h>
#include <errno.h>
#include <string.h>
#include "../BaseSocket.h"

#include <uxvcos.h>

namespace System {
namespace Win32 {

class UXVCOS_API Socket : public BaseSocket {
public:
  using BaseSocket::SocketType;

  Socket(SocketType type = SOCKET_UDP)
    : BaseSocket(type)
  {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(1, 1), &wsa)) {
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "WSAStartup() failed, %lu", static_cast<unsigned long>(GetLastError()));
    }
  }

  Socket(int socket, SocketType type)
    : BaseSocket(socket, type)
  {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(1, 1), &wsa)) {
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "WSAStartup() failed, %lu", static_cast<unsigned long>(GetLastError()));
    }
  }

  virtual ~Socket() {
    close();
  }

  using BaseSocket::operator=;
  using BaseSocket::operator==;

  bool create(unsigned short protocol = 0)
  {
	  if (socketType == SOCKET_PACKET) return false;

    socketId = ::socket(AF_INET, (socketType == SOCKET_TCP ? SOCK_STREAM : SOCK_DGRAM), 0);

	  if (socketId == 0) socketId = -1;
    return socketId != -1;
  }

  bool bind(const NetworkAddress& addr)
  {
    if (socketId < 0) return false;
          sockaddr_in local;

    if (!addr.valid) return false;

    local.sin_addr.s_addr = addr.ip;
    local.sin_port = addr.port;
    local.sin_family = AF_INET;

    int rc = ::bind(socketId, reinterpret_cast<sockaddr*>(&local), sizeof(local));
    if (rc != 0) return false;

    if (socketType == SOCKET_UDP) _eof = false;
    return true;
  }

  bool bind(const std::string device, unsigned short protocol = 0, EthernetAddress* address = 0)
  {
    if (socketId < 0) return false;
    return false;
  }

  bool bindAny(unsigned short port)
  {
    return bind(NetworkAddress::any(port));
  }

  bool listen()
  {
    if (socketId < 0) return false;
    int rc = ::listen(socketId, SOCKET_LISTEN_BACKLOG);
    return rc == 0;
  }

  bool accept(BaseSocket& acceptSocket, NetworkAddress& address)
  {
    if (socketId < 0) return false;

    sockaddr_in remote;
    int size = sizeof(remote);
    SOCKET id = ::accept(socketId, reinterpret_cast<sockaddr*>(&remote), &size);
    if (id == INVALID_SOCKET) {
      return false;
    }

    acceptSocket = Socket(id, SOCKET_TCP);
    acceptSocket.reset();

    address.ip = remote.sin_addr.s_addr;
    address.port = remote.sin_port;

    return true;
  }

  BaseSocket *accept(NetworkAddress& addr)
  {
    if (socketId < 0) return false;
    BaseSocket *socket = new Socket(SOCKET_TCP);
    if (accept(*socket, addr)) return socket;
    delete socket;
    return 0;
  }

  bool connect(const NetworkAddress& addr, bool nonBlocking = false)
  {
    if (socketId < 0) return false;
    sockaddr_in remote;

    if (!addr.valid) return false;

    remote.sin_addr.s_addr = addr.ip;
    remote.sin_port = addr.port;
    remote.sin_family = AF_INET;

    if (nonBlocking) {
      u_long iMode = 1;
      ::ioctlsocket(socketId, FIONBIO, &iMode);
    }

    int rc = ::connect(socketId, reinterpret_cast<sockaddr*>(&remote), sizeof(remote));
    if (rc != 0) return false;

    reset();

    if (nonBlocking) {
      u_long iMode = 0;
      ::ioctlsocket(socketId, FIONBIO, &iMode);
    }

    return true;
  }

  virtual bool connect(const EthernetAddress& addr) {
    if (socketId < 0) return false;
    reset();
    return false;
  }

  void close()
  {
    ::closesocket(socketId);
    _eof = true;
    _overflow = true;
    socketId = -1;
  }

  /*
  SocketRetVal _waitForReadable(int msec)
  {
    fd_set fds;
    FD_ZERO(&fds);
#pragma warning( disable : 4127 )
    FD_SET(socketId, &fds);
#pragma warning( default : 4127 )

    int rc;
    if (msec < 0) {
      rc = select(static_cast<int>(socketId) + 1, &fds, NULL, NULL, NULL);
    } else {
      struct timeval tv;
      tv.tv_sec = msec / 1000;
      tv.tv_usec = (msec % 1000) * 1000;
      rc = select(static_cast<int>(socketId) + 1, &fds, NULL, NULL, &tv);
    }
    if (rc == SOCKET_ERROR) {
      return IBaseSocket::failed;
    } else if (rc == 0) {
      return IBaseSocket::timeout;
    }

    return (FD_ISSET(socketId, &fds) != 0) ? IBaseSocket::success : IBaseSocket::timeout;
  }

  SocketRetVal _waitForWritable(int msec)
  {
    fd_set fds;
    FD_ZERO(&fds);
#pragma warning( disable : 4127 )
    FD_SET(socketId, &fds);
#pragma warning( default : 4127 )

    int rc;
    if (msec < 0) {
      rc = select(static_cast<int>(socketId) + 1, NULL, &fds, NULL, NULL);
    } else {
      struct timeval tv;
      tv.tv_sec = msec / 1000;
      tv.tv_usec = (msec % 1000) * 1000;
      rc = select(static_cast<int>(socketId) + 1, NULL, &fds, NULL, &tv);
    }
    if (rc == SOCKET_ERROR) {
      return IBaseSocket::failed;
    } else if (rc == 0) {
      return IBaseSocket::timeout;
    }

    return (FD_ISSET(socketId, &fds) != 0) ? IBaseSocket::success : IBaseSocket::timeout;
  }
  */

  bool send(const void* data, size_t size)
  {
    if (socketId < 0) return false;

    error(0);
    int bytesSend = ::send(socketId, static_cast<const char*>(data), size, 0);
    if (bytesSend == SOCKET_ERROR) {
      error(WSAGetLastError());
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "send failed [%s]", strerror(errno));
    } else if (bytesSend != (long)size) {
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "send did not write the correct length");
    }
    if (bytesSend == SOCKET_ERROR || bytesSend != (long)size) {
      return false;
    }
    return true;
  }

  ssize_t receive(void* data, size_t size, bool waitall = false, bool peek = false)
  {
    if (socketId < 0) return -1;

    int flags = 0;
    if (peek) {
      flags |= MSG_PEEK;
    }

    error(0);
    int bytesRecv = ::recv(socketId, static_cast<char*>(data), size, flags);
    if (bytesRecv == SOCKET_ERROR) {
      error(WSAGetLastError());
      if (_error == WSAEWOULDBLOCK) {
        error(0);
        bytesRecv = 0;
      } else {
        if (_error == WSAECONNRESET || _error == WSAEBADF || _error == WSAENOTCONN) close();
        return -1;
      }
    }

    return static_cast<ssize_t>(bytesRecv);
  }

  bool setBroadcast(bool enable)
  {
    if (socketId < 0) return false;
    int value = enable ? 1 : 0;

    if (setsockopt(socketId, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&value), sizeof(value)) == -1) {
		return false;
    }
	return true;
  }

  bool setReuseAddr(bool enable)
  {
    if (socketId < 0) return false;
    int value = enable ? 1 : 0;

    error(0);
    if (setsockopt(socketId, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&value), sizeof(value)) == SOCKET_ERROR) {
      error(WSAGetLastError());
      return false;
    }
	  return true;
  }

  bool setNonBlocking(bool enable)
  {
    if (socketId < 0) return false;
    u_long iMode = enable ? 1 : 0;
    return (::ioctlsocket(socketId, FIONBIO, &iMode) == 0);
  }

  ssize_t receiveFrom(void* data, size_t size, NetworkAddress& from, bool waitall = false, bool peek = false)
  {
    if (socketId < 0) return -1;
    sockaddr_in src;
    int srclen = sizeof(src);

    int flags = 0;
    if (peek) {
      flags |= MSG_PEEK;
    }

    error(0);
    int bytesRecv = ::recvfrom(socketId, static_cast<char*>(data), size, flags, reinterpret_cast<sockaddr*>(&src), &srclen);

    from.ip = inet_addr(inet_ntoa(src.sin_addr));
    from.port = src.sin_port;

    if (bytesRecv == SOCKET_ERROR) {
      error(WSAGetLastError());
      if (_error == WSAEWOULDBLOCK) {
        error(0);
        bytesRecv = 0;
      } else {
        if (_error == WSAECONNRESET || _error == WSAEBADF || _error == WSAENOTCONN) close();
        return -1;
      }
    }

    return static_cast<ssize_t>(bytesRecv);
  }

  ssize_t receiveFrom(void* data, size_t size, EthernetAddress& from, bool waitall = false, bool peek = false)
  {
    if (socketId < 0) return -1;
    return -1;
  }

  bool sendTo(const void* data, size_t size, const NetworkAddress& to)
  {
    error(false);
    if (socketId < 0) return false;
    sockaddr_in remote;

    if (!to.valid) return false;

    //memset(&remote,0,sizeof(sockaddr_in)); // zuerst alles auf //0 setzten
    remote.sin_addr.s_addr = to.ip;
    remote.sin_port = htons(static_cast<unsigned short>(to.port));
    remote.sin_family = AF_INET;

    error(0);
    int bytesSend = sendto(socketId, static_cast<const char*>(data), size, 0, reinterpret_cast<sockaddr*>(&remote), sizeof(remote));

    if (bytesSend == SOCKET_ERROR) {
      error(WSAGetLastError());
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "sendTo failed");
    } else if ((size_t) bytesSend != size) {
      // LOGPATH_ERROR("roboapp.platform.system.win32.BaseSocket", "sendTo did not send the correct length (%d instead of %u)", bytesSend, size);
    }
    if (bytesSend == SOCKET_ERROR || bytesSend != (long)size) {
      return false;
    }
    return true;
  }

  bool sendTo(const void* data, size_t size, const EthernetAddress& to) {
    if (socketId < 0) return false;
    return false;
  }

  bool wait(unsigned long msecs) {
    if (socketId < 0) return false;
    fd_set rfds;
    struct timeval tv;
    int ret;

    FD_ZERO(&rfds);
    FD_SET((SOCKET) socketId, &rfds);
    
    tv.tv_sec = msecs / 1000;
    tv.tv_usec = (msecs * 1000) % 1000000;

    error(0);
    ret = select(socketId + 1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */

    if (ret == -1) {
      error(errno);
    }

    return (ret > 0);
  }

  void flush() {
    char temp[256];
    size_t s = sizeof(temp);
    while(wait(0) && receive(temp, s, true));
  }

private:
  static const unsigned int SOCKET_LISTEN_BACKLOG = 0;
};

} // namespace Win32
} // namespace System

#endif // SYSTEM_WIN32_SOCKET_H
