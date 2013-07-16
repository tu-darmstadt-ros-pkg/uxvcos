#include "Error.h"

#include <sstream>
#include <cerrno>
#include <cstring>
#include <winsock2.h>

namespace System {

std::string Error::string() const {
  std::ostringstream s;
  s << std::strerror(_error) << " (" << _error << ")";
  return s.str();
}

Error Error::lastError() {
  return Error(errno ? errno : WSAGetLastError());
}

Error Error::setError(int error) {
  errno = error;
  return Error(error);
}

} // namespace System
