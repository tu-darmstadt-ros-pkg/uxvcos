#include "stream/PropertyStream.h"

#include <uxvcos/data/Streamable.h>
#include <uxvcos/data/TypeInfo.h>

PropertyInStream& PropertyInStream::compose(Data::Streamable& data) {
  fieldNames = data.getFieldNames();
  field = 0;
  
  RTT::log( RTT::Debug ) << "composing Streamable type " << (data.getTypeInfo() ? data.getTypeInfo()->getTypeName() : typeid(data).name()) << RTT::endlog();
  
  data << *this;
  return *this;
}
    
PropertyOutStream& PropertyOutStream::decompose(const Data::Streamable& data) {
  fieldNames = data.getFieldNames();
  field = 0;

  RTT::log( RTT::Debug ) << "decomposing Streamable type " << (data.getTypeInfo() ? data.getTypeInfo()->getTypeName() : typeid(data).name()) << RTT::endlog();

  data >> *this;
  return *this;
}

PropertyInStream& operator>>(PropertyInStream& in, Data::Streamable& data) {
  return in.compose(data);
}

PropertyOutStream& operator<<(PropertyOutStream& out, const Data::Streamable& data) {
  return out.decompose(data);
}
