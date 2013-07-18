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
