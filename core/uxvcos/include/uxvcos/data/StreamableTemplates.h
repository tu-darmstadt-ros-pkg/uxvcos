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

#ifndef DATA_STREAMABLETEMPLATES_H
#define DATA_STREAMABLETEMPLATES_H

#include "Streamable.h"
#include <vector>
#include <boost/array.hpp>

namespace Data {

template<typename T>
struct UXVCOS_API StreamableValue : public Streamable {
  T value;

  StreamableValue(const T& init = T()) : value(init) {}
  StreamableValue(const StreamableValue<T>& other) : Streamable(other), value(other.value) {}
  virtual ~StreamableValue() {}

  operator T() { return value; }
  T& operator=(const T& other) { value = other; }

  InStream& operator<<(InStream& in)
  {
    return in >> value;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << value;
  }
};

template<typename T, size_t N = 1>
struct UXVCOS_API StreamableArray : public Streamable {
  typedef boost::array<T,N> Array;
  Array value;

  StreamableArray(const T& init = T()) {
    value.assign(init);
  }
  StreamableArray(const StreamableArray<T,N>& other) : Streamable(other) {
    value = other.value;
  }
  virtual ~StreamableArray() {}

  operator Array() { return value; }
  Array& operator=(const Array& other) { value = other; }
  T& operator[](size_t i) { return value[i]; }
  const T& operator[](size_t i) const { return value[i]; }

  InStream& operator<<(InStream& in)
  {
    return in >> value;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << value;
  }
};

template <typename T>
struct UXVCOS_API StreamableVector : public Streamable
{
  typedef std::vector<T> Vector;
  Vector vector;

  StreamableVector(size_t n = 0, const T& init = T()) : vector(n, init) {}
  StreamableVector(const Vector& other) : vector(other) {}
  StreamableVector(const StreamableVector<T>& other) : Streamable(other), vector(other.vector) {}
  virtual ~StreamableVector() {}

  operator Vector() { return vector; }
  Vector& operator=(const Vector& other) { vector = other; }
  T& operator[](size_t i) { return vector[i]; }
  const T& operator[](size_t i) const { return vector[i]; }

  InStream& operator<<(InStream& in)
  {
    return in >> vector;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << vector;
  }
};

} // namespace Data

#endif // DATA_STREAMABLETEMPLATES_H
