#ifndef SYSTEM_XENOMAI_PIPE_H
#define SYSTEM_XENOMAI_PIPE_H

#include "Xenomai.h"
#include <native/pipe.h>
#include <string>

namespace System {
namespace Xenomai {

class Pipe {
public:
  Pipe(const std::string name = "", size_t poolsize = 0);
  Pipe(RT_PIPE rt_pipe);
  ~Pipe();
  void destroy();

  ssize_t read(void *buf, size_t size, long timeout = XENOMAI_DEFAULT_TIMEOUT_MS);
  ssize_t write(const void *buf, size_t size, int mode = P_NORMAL);
  ssize_t stream(const void *buf, size_t size);

  size_t getSize() { return poolsize; }

private:
  const std::string name;
  size_t poolsize;
  RT_PIPE  rt_pipe;
  bool created;
};

} // namespace Xenomai
} // namespace System

#endif // SYSTEM_XENOMAI_PIPE_H
