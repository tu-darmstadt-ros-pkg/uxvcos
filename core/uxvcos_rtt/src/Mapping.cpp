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

#include "Mapping.h"

#include <boost/lexical_cast.hpp>
#include <sstream>

namespace uxvcos {

std::string MappingBase::toString() const {
  if (this->size() == 0) return std::string();
  std::ostringstream ss;
  ss << (*this)[0];
  for(size_t i = 1; i < this->size(); ++i)
    ss << std::string::value_type(',') << (*this)[i];
  return ss.str();
}

bool MappingBase::fromString(const std::string& s) {
  std::istringstream ss(s);
  size_t i = 0;
  std::string text;

  while(std::getline(ss, text, std::string::value_type(','))) {
    size_t temp = boost::lexical_cast<size_t>(text);
    (*this)[i++] = temp;
  }
  for(; i < this->size(); ++i) (*this)[i] = 0;
  this->set_size(i);
  return true;
}

std::ostream& operator<<(std::ostream& os, const MappingBase& map) {
  return os << '[' << map.toString() << ']';
}

std::istream& operator>>(std::istream& is, MappingBase& map) {
  std::string s;
  if (!(is >> s) || !map.fromString(s)) is.setstate(std::ios_base::failbit);
  return is;
}

} // namespace uxvcos
