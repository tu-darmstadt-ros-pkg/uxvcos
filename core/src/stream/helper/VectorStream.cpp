#include "VectorStream.h"
#include <data/Streamable.h>

VectorInStream& operator>>(VectorInStream& in, Data::Streamable& data) {
  data << in;
  return in;
}

VectorOutStream& operator<<(VectorOutStream& out, const Data::Streamable& data) {
  data >> out;
  return out;
}
