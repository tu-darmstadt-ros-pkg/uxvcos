#ifndef SYSTEM_XENOMAI_SYSTEMCALL_H
#define SYSTEM_XENOMAI_SYSTEMCALL_H

#include <native/task.h>

#ifdef USE_RTNET
#include <rtnet.h>

#include <errno.h>
#include <sys/select.h>

namespace System {
namespace Xenomai {
class SystemCall {
public:

  static inline unsigned int sleep(unsigned int seconds) {
    RTIME nsecs = 1000000000ull * seconds;
    return (rt_task_sleep(nsecs) == 0) ? 0 : ~0;
  }

  static inline int open(const char *path, int oflag, ...) {
    int ret = rt_dev_open(path, oflag);
    return return_errno(ret);
  }

  static inline int socket(int domain, int type, int protocol) {
    int ret = rt_dev_socket(domain, type, protocol);
    return return_errno(ret);
  }

  static inline int close(int fd) {
    int ret;
    while((ret = rt_dev_close(fd)) == -EAGAIN) sleep(1);
    return return_errno(ret);
  }

  static inline ssize_t read(int fd, void *buf, size_t n) {
    ssize_t ret = rt_dev_read(fd, buf, n);
    return return_errno(ret);
  }

  static inline ssize_t write(int fd, const void *buf, size_t n) {
    ssize_t ret = rt_dev_write(fd, buf, n);
    return return_errno(ret);
  }

  static inline int bind(int fd, const struct sockaddr* addr, socklen_t len) {
    int ret = rt_dev_bind(fd, addr, len);
    return return_errno(ret);
  }

  static inline int getsockname(int fd, struct sockaddr* addr, socklen_t *len) {
    int ret = rt_dev_getsockname(fd, addr, len);
    return return_errno(ret);
  }

  static inline int connect(int fd, const struct sockaddr* addr, socklen_t len) {
    int ret = rt_dev_connect(fd, addr, len);
    return return_errno(ret);
  }

  static inline int getpeername(int fd, struct sockaddr* addr, socklen_t *len) {
    int ret = rt_dev_getpeername(fd, addr, len);
    return return_errno(ret);
  }

  static inline ssize_t send(int fd, const void *buf, size_t n, int flags) {
    ssize_t ret = rt_dev_send(fd, buf, n, flags);
    return return_errno(ret);
  }

  static inline ssize_t recv(int fd, void *buf, size_t n, int flags) {
    ssize_t ret = rt_dev_recv(fd, buf, n, flags);
    return return_errno(ret);
  }

  static inline ssize_t sendto(int fd, const void *buf, size_t n, int flags, const struct sockaddr* addr, socklen_t addr_len) {
    ssize_t ret = rt_dev_sendto(fd, buf, n, flags, addr, addr_len);
    return return_errno(ret);
  }

  static inline ssize_t recvfrom(int fd, void *buf, size_t n, int flags, struct sockaddr* addr, socklen_t *addr_len) {
    ssize_t ret = rt_dev_recvfrom(fd, buf, n, flags, addr, addr_len);
    return return_errno(ret);
  }

  static inline ssize_t sendmsg(int fd, const struct msghdr *message, int flags) {
    ssize_t ret = rt_dev_sendmsg(fd, message, flags);
    return return_errno(ret);
  }

  static inline ssize_t recvmsg(int fd, struct msghdr *message, int flags) {
    ssize_t ret = rt_dev_recvmsg(fd, message, flags);
    return return_errno(ret);
  }

  static inline int getsockopt(int fd, int level, int optname, void* optval, socklen_t* optlen) {
    int ret = rt_dev_getsockopt(fd, level, optname, optval, optlen);
    return return_errno(ret);
  }

  static inline int setsockopt (int fd, int level, int optname, const void *optval, socklen_t optlen) {
    int ret = rt_dev_setsockopt(fd, level, optname, optval, optlen);
    return return_errno(ret);
  }

  static inline int listen (int fd, int n) {
    int ret = rt_dev_listen(fd, n);
    return return_errno(ret);
  }

  static inline int accept (int fd, struct sockaddr* addr, socklen_t* addr_len) {
    int ret = rt_dev_accept(fd, addr, addr_len);
    return return_errno(ret);
  }
  
  static inline int shutdown(int fd, int how) {
    int ret = rt_dev_shutdown(fd, how);
    return return_errno(ret);
  }
  
  template<typename T>
  static inline int ioctl(int fd, unsigned long int request, T value) {
    int ret = rt_dev_ioctl(fd, request, value);
    return return_errno(ret);
  }
  
  static inline int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout) {
    int64_t timeout64;
    int socketId = -1;

    for(int i = 0; i < nfds; ++i) {
      if (FD_ISSET(i, readfds)) {
        socketId = i;
        break;
      }
    }

    if (!timeout) {
      timeout64 = 0;
    } else if (timeout->tv_sec == 0 && timeout->tv_usec == 0) {
      timeout64 = -1;
    } else {
      timeout64 = (int64_t) timeout->tv_sec * 1000000000ull + (int64_t) timeout->tv_usec * 1000ull;
    }

    int ret = rt_dev_ioctl(socketId, RTNET_RTIOC_TIMEOUT, &timeout64);
    if (ret < 0) return return_errno(ret);

    ret = 1; // trigger read system call and wait there

  //   int ret = rt_dev_read(socketId, 0, 0);
  //   if (ret == -ETIMEDOUT || ret == -EAGAIN) return return_errno(0);
  
    return return_errno(ret);
  }
  
private:
  template<typename T>
  static inline T return_errno(T ret) {
    errno = 0;
    if (ret < 0) {
      errno = -ret;
      return -1;
    }
    return ret;
  }

  static inline int rtnet_rtioc_timeout(int fd, int64_t timeout64) {
    return rt_dev_ioctl(fd, RTNET_RTIOC_TIMEOUT, &timeout64);
  }

}; // class SystemCall
} // namespace Xenomai
} // namespace System

#else // USE_RTNET

#include "../unix/SystemCall.h"

namespace System {
namespace Xenomai {
  typedef Unix::SystemCall SystemCall;
} // namespace Xenomai
} // namespace System

#endif // USE_RTNET
#endif // SYSTEM_XENOMAI_SYSTEMCALL_H
