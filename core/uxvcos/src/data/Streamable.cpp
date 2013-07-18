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
