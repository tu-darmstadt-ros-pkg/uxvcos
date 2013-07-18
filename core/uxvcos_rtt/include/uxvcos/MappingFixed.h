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

#ifndef UXVCOS_MAPPING_H
#define UXVCOS_MAPPING_H

#include <rtt/TaskContext.hpp>
#include <vector>

namespace uxvcos {

class MappingBase {
public:
  virtual size_t& operator[](size_t index) = 0;
  virtual size_t operator[](size_t index) const = 0;
  virtual size_t size() const = 0;

  std::string toString() const;
  bool fromString(const std::string& s);
};

template<size_t n>
class Mapping : public MappingBase {
public:
  Mapping()
    : map(n)
  {
    for(size_t i = 0; i < n; i++) map[i] = i;
  }

  Mapping<n>& operator=(const Mapping<n> &other) {
    this->map = other.map;
    return *this;
  }

  size_t& operator[](size_t index) {
    return map[index];
  }

  size_t operator[](size_t index) const {
    return map[index];
  }

  size_t size() const {
    return static_cast<size_t>(n);
  }

  template <typename T>
  T* operator()(T *array) const {
    T temp[n];
    for(size_t i = 0; i < n; i++) temp[i] = array[i];
    for(size_t i = 0; i < n; i++) array[i] = temp[map[i]];
    return array;
  }

  template <typename T>
  T* invert(T *array) const {
    T temp[n];
    for(size_t i = 0; i < n; i++) temp[i] = array[i];
    for(size_t i = 0; i < n; i++) array[map[i]] = temp[i];
    return array;
  }

protected:
  typedef std::vector<size_t> Table;
  Table map;
};

typedef Mapping<3> Mapping3;
std::ostream& operator<<(std::ostream& os, const MappingBase& map);
std::istream& operator>>(std::istream& is, MappingBase& map);

} // namespace uxvcos

#endif // UXVCOS_MAPPING_H
