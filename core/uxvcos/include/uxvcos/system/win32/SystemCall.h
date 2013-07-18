#ifndef SYSTEM_WIN32_SYSTEMCALL_H
#define SYSTEM_WIN32_SYSTEMCALL_H

#include <fcntl.h>      /* Needed only for _O_RDWR definition */
#include <io.h>
#include <stdlib.h>
#include <stdio.h>
#include <share.h>
#include <winsock2.h>
#include <windows.h>

namespace System {

typedef unsigned int mode_t;
typedef int ssize_t;
typedef int socklen_t;
struct msghdr;

namespace Win32 {

class SystemCall {
public:

  static inline unsigned int sleep(unsigned int seconds) {
    ::Sleep(seconds*1000);
    return 0;
  }

  static inline unsigned int usleep(unsigned int useconds) {
    ::Sleep(static_cast<DWORD>(useconds/1000.0+.5));
    return 0;
  }

  static inline int open(const char *pathname, int flags) {
    return _open(pathname, flags);
  }

  static inline int open(const char *pathname, int flags, mode_t mode) {
    return _open(pathname, flags, mode);
  }

  static inline int socket(int domain, int type, int protocol) {
    return ::socket(domain, type, protocol);
  }

  static inline int close(int fd) {
    return _close(fd);
  }

  static inline ssize_t read(int fd, void *buf, size_t n) {
    return _read(fd, buf, n);
  }

  static inline ssize_t write(int fd, const void *buf, size_t n) {
    return _write(fd, buf, n);
  }

  static inline int bind(int fd, const struct sockaddr* addr, socklen_t len) {
    return ::bind(fd, addr, len);
  }

  static inline int getsockname(int fd, struct sockaddr* addr, socklen_t *len) {
    return ::getsockname(fd, addr, len);
  }

  static inline int connect(int fd, const struct sockaddr* addr, socklen_t len) {
    return ::connect(fd, addr, len);
  }

  static inline int getpeername(int fd, struct sockaddr* addr, socklen_t *len) {
    return ::getpeername(fd, addr, len);
  }

  static inline ssize_t send(int fd, const void *buf, size_t n, int flags) {
    return ::send(fd, static_cast<const char *>(buf), n, flags);
  }

  static inline ssize_t recv(int fd, void *buf, size_t n, int flags) {
    return ::recv(fd, static_cast<char *>(buf), n, flags);
  }

  static inline ssize_t sendto(int fd, const void *buf, size_t n, int flags, const struct sockaddr* addr, socklen_t addr_len) {
    return ::sendto(fd, static_cast<const char *>(buf), n, flags, addr, addr_len);
  }

  static inline ssize_t recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr* addr, socklen_t *addr_len) {
    return ::recvfrom(fd, static_cast<char *>(buf), n, flags, addr, addr_len);
  }

  static inline ssize_t sendmsg(int fd, const struct msghdr *message, int flags) {
    return -1;
  }

  static inline ssize_t recvmsg(int fd, struct msghdr *message, int flags) {
    return -1;
  }

  static inline int getsockopt(int fd, int level, int optname, void* optval, socklen_t* optlen) {
    return ::getsockopt(fd, level, optname, static_cast<char *>(optval), optlen);
  }

  static inline int setsockopt (int fd, int level, int optname, const void *optval, socklen_t optlen) {
    return ::setsockopt(fd, level, optname, static_cast<const char *>(optval), optlen);
  }

  static inline int listen (int fd, int n) {
    return ::listen(fd, n);
  }

  static inline int accept (int fd, struct sockaddr* addr, socklen_t* addr_len) {
    return ::accept(fd, addr, addr_len);
  }

  static inline int shutdown(int fd, int how) {
    return ::shutdown(fd, how);
  }

  template<typename T>
  static inline int ioctl(int fd, unsigned long int request, T value) {
    return ::ioctl(fd, request, value);
  }

  static inline int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout) {
    return ::select(nfds, readfds, writefds, exceptfds, timeout);
  }

}; // class SystemCall
} // namespace Win32
} // namespace System

#endif // SYSTEM_WIN32_SYSTEMCALL_H
