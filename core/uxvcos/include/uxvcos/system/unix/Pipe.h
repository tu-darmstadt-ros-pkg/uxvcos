#ifndef SYSTEM_UNIX_PIPE_H
#define SYSTEM_UNIX_PIPE_H

#include <string>
#include <stddef.h>

#include <uxvcos/uxvcos.h>

namespace System {
namespace Unix {

class UXVCOS_API Pipe {
public:
  Pipe(const std::string name = "");
  ~Pipe();
  void destroy();

  ssize_t read(void *buf, size_t size, long timeout = 1000);
  ssize_t write(const void *buf, size_t size);
  ssize_t stream(const void *buf, size_t size);

private:
  static const std::string path;
  const std::string name;
  int fd;
  bool created;
};

} // namespace Unix
} // namespace System

#endif // SYSTEM_UNIX_PIPE_H
