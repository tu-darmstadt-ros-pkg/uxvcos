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

#ifndef SYSTEM_UNIX_FILESTREAM_H
#define SYSTEM_UNIX_FILESTREAM_H

#include <fstream>
#include <string>

#include <stream/Stream.h>
#include <system/BaseFileStream.h>

namespace System {
namespace Unix {

class UXVCOS_API InFileStream : public InStream, public BaseFileStream {
public:
  InFileStream();
  InFileStream(const std::string filename);
  virtual ~InFileStream();

  virtual element_t get();
  virtual InStream& read(void *destination, BaseStream::size_t n);
  virtual size_t readsome(void *destination, BaseStream::size_t n);
  virtual size_t size() const;

  virtual bool open();
  virtual bool open(const std::string filename);
  virtual void close();

  virtual bool eof() const {
    return in.eof();
  }

  virtual bool isOpen() const {
    return in.is_open();
  }

  virtual bool good() const {
    return isOpen() && InStream::good();
  }


protected:
  std::ifstream in;
  std::string filename;
};

class UXVCOS_API OutFileStream : public OutStream, public BaseFileStream {
public:
  OutFileStream();
  OutFileStream(const std::string filename);
  virtual ~OutFileStream();

  virtual OutStream& write(const void *source, BaseStream::size_t size);
  virtual OutStream& put(unsigned char c);

  virtual bool open();
  virtual bool open(const std::string filename);
  virtual void close();

  virtual bool overflow() const {
    return out.fail();
  }

  virtual bool isOpen() const {
    return out.is_open();
  }

  virtual bool good() const {
    return isOpen() && OutStream::good();
  }

protected:
  std::ofstream out;
  std::string filename;
};

} // namespace Unix
} // namespace System

#endif // SYSTEM_UNIX_FILESTREAM_H
