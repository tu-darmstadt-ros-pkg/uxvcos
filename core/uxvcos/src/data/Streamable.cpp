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

#include "data/Streamable.h"
#include "data/TypeRegistry.h"
#include "data/TypeInfo.h"

#include "stream/StreamMapper.h"
#include "data/StreamableFormatter.h"

#include <typeinfo>
#include <sstream>

namespace Data {

Streamable * const Streamable::INVALID = reinterpret_cast<Streamable *>(0);
Streamable * const Streamable::BREAK = reinterpret_cast<Streamable *>(-1);

std::string Streamable::getName() const
{
  TypeInfo *typeInfo = getTypeInfo();
  return typeInfo ? typeInfo->getTypeName() : "unknown";
}

TypeInfo* Streamable::getTypeInfo() const {
  if (_key) return types()->getTypeInfo(_key);
  return types()->getTypeInfoById(&typeid(*this));
}

Key Streamable::getKey() const
{
  if (_key) return _key;
  const TypeInfo* typeInfo = getTypeInfo();
  if (typeInfo) return typeInfo->getKey();
  return 0;
}

void Streamable::setKey(Key key)
{
  _key = key;
}

std::istream& Streamable::operator<<(std::istream& is) {
  InStreamMapper mapper(is);
  (*this) << mapper;
  return is;
}

std::ostream& Streamable::operator>>(std::ostream& os) const {
  const std::string& format = getFormat();
  if (format.empty()) {
    OutStreamMapper mapper(os);
    os << "[";
    (*this) >> mapper;
    os << "]";
    // os << " (t = " << getTimestamp().toSec() << ")";
  } else {
    StreamableFormatter formatter(format);
    OutArchiveMapper<StreamableFormatter> out(formatter);
    (*this) >> out;
    os << formatter;
  }
  return os;
}

} // namespace Data
