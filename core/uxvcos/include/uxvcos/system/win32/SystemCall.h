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
