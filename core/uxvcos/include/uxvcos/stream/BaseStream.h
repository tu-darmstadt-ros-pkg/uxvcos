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

#ifndef STREAM_BASESTREAM_H
#define STREAM_BASESTREAM_H

#include <uxvcos/uxvcos.h>

class UXVCOS_API BaseStream
{
public:
  typedef unsigned char element_t;
  typedef unsigned int size_t;
  typedef int ssize_t;
  typedef unsigned int index_t;

  BaseStream() : _error(0) {}
  virtual ~BaseStream() {}

  virtual bool good() const {
    return _error == 0;
  }

  virtual operator void *() {
    return reinterpret_cast<void*>(good());
  }

  virtual bool open() {
    return true;
  }

  virtual void close() {
  }

  virtual void error(const int& i) {
    _error = i;
  }

  virtual void error(const bool& e) {
    _error = (!e ? 0 : (!_error ? -1 : _error));
  }

  virtual int error() const {
    return _error;
  }

  virtual void fail() {
    error(true);
  }

  virtual void reset() {
    error(false);
  }

  virtual const char *strerror() const {
    return strerror(error());
  }

  virtual const char *strerror(const int err) const {
    return 0;
  }

protected:
  int _error;
};

class OutStream;
class InStream;
class Stream;

#endif // STREAM_BASESTREAM_H
