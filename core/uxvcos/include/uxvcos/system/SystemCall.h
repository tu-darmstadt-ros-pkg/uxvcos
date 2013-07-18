#ifndef SYSTEM_SYSTEMCALL_H
#define SYSTEM_SYSTEMCALL_H

#ifdef USE_XENOMAI
  #include "xenomai/SystemCall.h"
  #define SYSTEM_SYSTEMCALL_IMPL Xenomai::SystemCall
#elif SYSTEM_UNIX
  #include "unix/SystemCall.h"
  #define SYSTEM_SYSTEMCALL_IMPL Unix::SystemCall
#elif SYSTEM_WIN32
  #include "win32/SystemCall.h"
  #define SYSTEM_SYSTEMCALL_IMPL Win32::SystemCall
#endif

namespace System {
namespace SystemCall {

static inline unsigned int sleep(unsigned int seconds) {
  return SYSTEM_SYSTEMCALL_IMPL::sleep(seconds);
}

static inline unsigned int usleep(unsigned int useconds) {
  return SYSTEM_SYSTEMCALL_IMPL::usleep(useconds);
}

static inline int open(const char *pathname, int flags) {
  return SYSTEM_SYSTEMCALL_IMPL::open(pathname, flags);
}

static inline int open(const char *pathname, int flags, mode_t mode) {
 return SYSTEM_SYSTEMCALL_IMPL::open(pathname, flags, mode);
}

static inline int socket(int domain, int type, int protocol) {
  return SYSTEM_SYSTEMCALL_IMPL::socket(domain, type, protocol);
}

static inline int close(int fd) {
  return SYSTEM_SYSTEMCALL_IMPL::close(fd);
}

static inline ssize_t read(int fd, void *buf, size_t n) {
  return SYSTEM_SYSTEMCALL_IMPL::read(fd, buf, n);
}

static inline ssize_t write(int fd, const void *buf, size_t n) {
  return SYSTEM_SYSTEMCALL_IMPL::write(fd, buf, n);
}

static inline int bind(int fd, const struct sockaddr* addr, socklen_t len) {
  return SYSTEM_SYSTEMCALL_IMPL::bind(fd, addr, len);
}

static inline int getsockname(int fd, struct sockaddr* addr, socklen_t *len) {
  return SYSTEM_SYSTEMCALL_IMPL::getsockname(fd, addr, len);
}

static inline int connect(int fd, const struct sockaddr* addr, socklen_t len) {
  return SYSTEM_SYSTEMCALL_IMPL::connect(fd, addr, len);
}

static inline int getpeername(int fd, struct sockaddr* addr, socklen_t *len) {
  return SYSTEM_SYSTEMCALL_IMPL::getpeername(fd, addr, len);
}

static inline ssize_t send(int fd, const void *buf, size_t n, int flags) {
  return SYSTEM_SYSTEMCALL_IMPL::send(fd, buf, n, flags);
}

static inline ssize_t recv(int fd, void *buf, size_t n, int flags) {
  return SYSTEM_SYSTEMCALL_IMPL::recv(fd, buf, n, flags);
}

static inline ssize_t sendto(int fd, const void *buf, size_t n, int flags, const struct sockaddr* addr, socklen_t addr_len) {
  return SYSTEM_SYSTEMCALL_IMPL::sendto(fd, buf, n, flags, addr, addr_len);
}

static inline ssize_t recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr* addr, socklen_t *addr_len) {
  return SYSTEM_SYSTEMCALL_IMPL::recvfrom(fd, buf, n, flags, addr, addr_len);
}

static inline ssize_t sendmsg(int fd, const struct msghdr *message, int flags) {
  return SYSTEM_SYSTEMCALL_IMPL::sendmsg(fd, message, flags);
}

static inline ssize_t recvmsg(int fd, struct msghdr *message, int flags) {
  return SYSTEM_SYSTEMCALL_IMPL::recvmsg(fd, message, flags);
}

static inline int getsockopt(int fd, int level, int optname, void* optval, socklen_t* optlen) {
  return SYSTEM_SYSTEMCALL_IMPL::getsockopt(fd, level, optname, optval, optlen);
}

static inline int setsockopt (int fd, int level, int optname, const void *optval, socklen_t optlen) {
  return SYSTEM_SYSTEMCALL_IMPL::setsockopt(fd, level, optname, optval, optlen);
}

static inline int listen (int fd, int n) {
  return SYSTEM_SYSTEMCALL_IMPL::listen(fd, n);
}

static inline int accept (int fd, struct sockaddr* addr, socklen_t* addr_len) {
  return SYSTEM_SYSTEMCALL_IMPL::accept(fd, addr, addr_len);
}

static inline int shutdown(int fd, int how) {
  return SYSTEM_SYSTEMCALL_IMPL::shutdown(fd, how);
}

template<typename T>
static inline int ioctl(int fd, unsigned long int request, T value) {
  return SYSTEM_SYSTEMCALL_IMPL::ioctl(fd, request, value);
}

static inline int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout) {
  return SYSTEM_SYSTEMCALL_IMPL::select(nfds, readfds, writefds, exceptfds, timeout);
}

} // namespace SystemCall
} // namespace System

#endif // SYSTEM_SYSTEMCALL_H
