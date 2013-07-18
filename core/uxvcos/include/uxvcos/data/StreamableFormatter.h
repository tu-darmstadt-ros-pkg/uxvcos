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

#ifndef DATA_STREAMABLEFORMATTER_H
#define DATA_STREAMABLEFORMATTER_H

#include <boost/format.hpp>
#include <vector>

namespace Data {

class StreamableFormatter : public boost::format {
public:
  StreamableFormatter(const std::string &format) : boost::format(format) { exceptions(0); }
  StreamableFormatter(const char *format) : boost::format(format) { exceptions(0); }

  template <typename T> StreamableFormatter& operator&(const T &value) {
    return print(value);
  }

  template <typename T> StreamableFormatter& print(const T &value);
  template <typename T> StreamableFormatter& print(const T *value);
  template <typename T> StreamableFormatter& print(const std::vector<T> &value);

  void fail() {}
};

// general format function
template <typename T>
StreamableFormatter& StreamableFormatter::print(const T &value) {
  *this % value;
  return *this;
}

// specialization for pointers
template <typename T>
StreamableFormatter& StreamableFormatter::print(const T *value) {
  *this % *value;
  return *this;
}

// specialization for vector types
template <typename T>
StreamableFormatter& StreamableFormatter::print(const std::vector<T> &value) {
  for (typename std::vector<T>::const_iterator it = value.begin(); it != value.end(); ++it)
    *this % (*it);
  return *this;
}

} // namespace Data

#endif // DATA_STREAMABLEFORMATTER_H
