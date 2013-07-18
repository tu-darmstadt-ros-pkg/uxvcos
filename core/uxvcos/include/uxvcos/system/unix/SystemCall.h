#ifndef SYSTEM_UNIX_SYSTEMCALL_H
#define SYSTEM_UNIX_SYSTEMCALL_H

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>

namespace System {
namespace Unix {
class SystemCall {
public:

  static inline unsigned int sleep(unsigned int seconds) {
    return ::sleep(seconds);
  }

  static inline unsigned int usleep(unsigned int useconds) {
    return ::usleep(useconds);
  }

  static inline int open(const char *pathname, int flags) {
    return ::open(pathname, flags);
  }

  static inline int open(const char *pathname, int flags, mode_t mode) {
    return ::open(pathname, flags, mode);
  }

  static inline int socket(int domain, int type, int protocol) {
    return ::socket(domain, type, protocol);
  }

  static inline int close(int fd) {
    return ::close(fd);
  }

  static inline ssize_t read(int fd, void *buf, size_t n) {
    return ::read(fd, buf, n);
  }

  static inline ssize_t write(int fd, const void *buf, size_t n) {
    return ::write(fd, buf, n);
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
    return ::send(fd, buf, n, flags);
  }

  static inline ssize_t recv(int fd, void *buf, size_t n, int flags) {
    return ::recv(fd, buf, n, flags);
  }

  static inline ssize_t sendto(int fd, const void *buf, size_t n, int flags, const struct sockaddr* addr, socklen_t addr_len) {
    return ::sendto(fd, buf, n, flags, addr, addr_len);
  }

  static inline ssize_t recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr* addr, socklen_t *addr_len) {
    return ::recvfrom(fd, buf, n, flags, addr, addr_len);
  }

  static inline ssize_t sendmsg(int fd, const struct msghdr *message, int flags) {
    return ::sendmsg(fd, message, flags);
  }

  static inline ssize_t recvmsg(int fd, struct msghdr *message, int flags) {
    return ::recvmsg(fd, message, flags);
  }

  static inline int getsockopt(int fd, int level, int optname, void* optval, socklen_t* optlen) {
    return ::getsockopt(fd, level, optname, optval, optlen);
  }

  static inline int setsockopt (int fd, int level, int optname, const void *optval, socklen_t optlen) {
    return ::setsockopt(fd, level, optname, optval, optlen);
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
} // namespace Unix
} // namespace System

#endif // SYSTEM_UNIX_SYSTEMCALL_H
