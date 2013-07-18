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

#include "Pipe.h"

// #include <system/unix/backtrace.h>
// #include <system/Exception.h>

namespace System {
namespace Xenomai {

  Pipe::Pipe(const std::string name, size_t poolsize) : name(name), poolsize(poolsize), created(false)
  {
    int ret = 0;

    if ((ret = rt_pipe_create(&rt_pipe, !name.empty() ? name.c_str() : 0, P_MINOR_AUTO, poolsize)) < 0) {
      // throw System::Exception(ret, "rt_pipe_create()");
      return;
    }

    created = true;
  }

  Pipe::Pipe(RT_PIPE rt_pipe) : rt_pipe(rt_pipe)
  {
  }

  ssize_t Pipe::read(void *buf, size_t size, long timeout) {
    if (!created) return -EINVAL;
    return rt_pipe_read(&rt_pipe, buf, size, milliseconds(timeout));
  }
  ssize_t Pipe::write(const void *buf, size_t size, int mode) {
    if (!created) return -EINVAL;
    return rt_pipe_write(&rt_pipe, buf, size, mode);
  }
  ssize_t Pipe::stream(const void *buf, size_t size) {
    if (!created) return -EINVAL;
    return rt_pipe_stream(&rt_pipe, buf, size);
  }

  Pipe::~Pipe()
  {
    destroy();
  }

  void Pipe::destroy() {
    if (created) rt_pipe_delete(&rt_pipe);
  }

} // namespace Xenomai
} // namespace System
