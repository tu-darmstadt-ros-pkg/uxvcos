#ifndef SYSTEM_ERROR_H
#define SYSTEM_ERROR_H

#include <string>
#include <iostream>

#include <uxvcos.h>

namespace System {

class UXVCOS_API Error {
  public:
    Error(int error) : _error(error) {}

    operator int&() { return _error; }
    operator void*() const { return (void*)(_error != 0); }
    
    void set(int error) { _error = error; }
    void reset()        { _error = 0; }
    
    std::string string() const;
    int error() const { return _error; }

    static Error lastError();
    static Error setError(int error);
  private:
    int _error;
};

static inline Error lastError()         { return Error::lastError(); }
static inline Error setError(int error) { return Error::setError(error); }

static inline std::ostream& operator<<(std::ostream& os, const Error& error) {
  return os << error.string();
}

} // namespace System

#ifdef SYSTEM_UNIX
  #include "unix/Error.h"
#endif

#ifdef SYSTEM_WIN32
  #include "win32/Error.h"
#endif

#endif // SYSTEM_ERROR_H
