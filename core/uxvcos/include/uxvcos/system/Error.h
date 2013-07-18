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

#ifndef SYSTEM_ERROR_H
#define SYSTEM_ERROR_H

#include <string>
#include <iostream>

#include <uxvcos/uxvcos.h>

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
