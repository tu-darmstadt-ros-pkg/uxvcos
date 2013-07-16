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
