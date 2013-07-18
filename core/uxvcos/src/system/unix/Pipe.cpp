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

#include "system/Pipe.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <errno.h>

// #include <system/Exception.h>

namespace System {
namespace Unix {
  const std::string Pipe::path("");

  Pipe::Pipe(const std::string name) : name(name), fd(-1), created(false)
  {
    int ret = 0;

    if (!path.empty()) mkdir(path.c_str(), 0777);
    ret = mkfifo((path + name).c_str(), 0666);
    if (ret != 0) {
      // throw System::Exception(ret, "mkfifo() returned an error");
      return;
    }
    created = true;

    fd = open((path + name).c_str(), O_RDWR | O_NONBLOCK);
  }

  ssize_t Pipe::read(void *buf, size_t size, long timeout) {
    if (fd < 0) return -EINVAL;

    if (timeout >= 0) {
      struct timeval _timeval;
      _timeval.tv_sec = timeout / 1000;
      _timeval.tv_usec = timeout % 1000;
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(fd, &readfds);

      int ret = select(fd + 1, &readfds, 0, 0, &_timeval);
      if (ret != 1) return 0;
    }

    return ::read(fd, buf, size);
  }

  ssize_t Pipe::write(const void *buf, size_t size) {
    if (fd < 0) return -EINVAL;
    return ::write(fd, buf, size);
  }

  ssize_t Pipe::stream(const void *buf, size_t size) {
    if (fd < 0) return -EINVAL;
    return ::write(fd, buf, size);
  }

  Pipe::~Pipe()
  {
    destroy();
  }

  void Pipe::destroy() {
    if (fd > 0) close(fd);
    if (created) unlink((path + name).c_str());
    created = false;
    fd = -1;
  }

} // namespace Unix
} // namespace System
