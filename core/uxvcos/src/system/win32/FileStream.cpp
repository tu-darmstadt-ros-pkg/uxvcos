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

#include "FileStream.h"
#include <limits.h>

namespace System {
namespace Win32 {

InFileStream::InFileStream()
{
}

InFileStream::InFileStream(const std::string filename)
  : filename(filename)
{
}

InFileStream::~InFileStream()
{
}

BaseStream::element_t InFileStream::get() {
  return static_cast<element_t>(in.get());
}

InStream& InFileStream::read(void *destination, BaseStream::size_t n) {
  in.read(static_cast<char *>(destination), n);
  return *this;
}

BaseStream::size_t InFileStream::readsome(void *destination, BaseStream::size_t n) {
  in.read(static_cast<char *>(destination), n);
  return in.gcount();
}

BaseStream::size_t InFileStream::size() const {
  return UINT_MAX;

  // if (!in.rdbuf()) return 0;
  // if (in.rdbuf()->in_avail() < 0) return 0;
  // return in.rdbuf()->in_avail();
}

bool InFileStream::open() {
  if (isOpen()) return true;
  in.clear();
  in.open(filename.c_str(), std::ios::in | std::ios::binary);

  return in.good();
}

bool InFileStream::open(const std::string filename) {
  close();
  this->filename = filename;
  return open();
}

void InFileStream::close() {
  in.close();
}

OutFileStream::OutFileStream()
{
}

OutFileStream::OutFileStream(const std::string filename)
  : filename(filename)
{
}

OutFileStream::~OutFileStream()
{
}

OutStream& OutFileStream::write(const void *source, BaseStream::size_t size) {
  out.write(static_cast<const char *>(source), size);
  return *this;
}

OutStream& OutFileStream::put(unsigned char c) {
  out.put(c);
  return *this;
}

bool OutFileStream::open() {
  if (isOpen()) return true;
  out.clear();
  out.open(filename.c_str(), std::ios::out | std::ios::binary);
  return out.good();
}

bool OutFileStream::open(const std::string filename) {
  close();
  this->filename = filename;
  return open();
}

void OutFileStream::close() {
  out.close();
}

} // namespace Win32
} // namespace System
