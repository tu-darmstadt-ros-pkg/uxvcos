#ifndef DATA_UNKNOWNTYPEINFO_H
#define DATA_UNKNOWNTYPEINFO_H

#include "TypeInfo.h"

namespace Data {

class UnknownTypeInfo : public TypeInfo
{
public:
  UnknownTypeInfo() : TypeInfo("Unknown") {}
  static const TypeInfo *Instance() { return instance; }

private:
  static const TypeInfo *instance;
};

} // namespace Data

#endif // DATA_UNKNOWNTYPEINFO_H
