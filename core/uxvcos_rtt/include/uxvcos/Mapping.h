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
  static const size_t VARIABLE = 0;

  virtual ~MappingBase() {}
  virtual size_t& operator[](size_t index) = 0;
  virtual const size_t& operator[](size_t index) const = 0;
  virtual size_t size() const = 0;
  virtual void set_size(size_t size) = 0;

  std::string toString() const;
  bool fromString(const std::string& s);
};

class Mapping : public MappingBase {
public:
  Mapping(size_t size = VARIABLE)
    : _size(size)
    , map(size)
  {
    for(size_t i = 0; i < map.size(); i++) map[i] = i;
  }
  virtual ~Mapping() {}

  Mapping& operator=(const Mapping &other) {
    this->_size = other._size;
    this->map = other.map;
    return *this;
  }

  size_t& operator[](size_t index) {
    if (_size == VARIABLE && index >= map.size()) map.resize(index+1);
    return map[index];
  }

  const size_t& operator[](size_t index) const {
    return map[index];
  }

  size_t size() const {
    return static_cast<size_t>(map.size());
  }

  void set_size(size_t size) {
    if (_size != VARIABLE) return;
    _size = size;
    map.resize(_size);
  }

  template <typename T>
  T* operator()(T *array) const {
    T temp[map.size()];
    for(size_t i = 0; i < map.size(); i++) temp[i] = array[i];
    for(size_t i = 0; i < map.size(); i++) array[i] = temp[map[i]];
    return array;
  }

  template <typename T>
  T* invert(T *array) const {
    T temp[map.size()];
    for(size_t i = 0; i < map.size(); i++) temp[i] = array[i];
    for(size_t i = 0; i < map.size(); i++) array[map[i]] = temp[i];
    return array;
  }

private:
  size_t _size;

  typedef std::vector<size_t> Table;
  Table map;
};

std::ostream& operator<<(std::ostream& os, const MappingBase& map);
std::istream& operator>>(std::istream& is, MappingBase& map);

} // namespace uxvcos

#endif // UXVCOS_MAPPING_H
