#ifndef UXVCOS_MAPPINGTYPEINFO_HPP
#define UXVCOS_MAPPINGTYPEINFO_HPP

#include "Mapping.h"

#include <rtt/types/TemplateTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <string>
#include <sstream>

namespace uxvcos {

struct MappingTypeInfo : public RTT::types::TemplateTypeInfo<Mapping,true>
{
  MappingTypeInfo(const std::string& name = "Mapping") : RTT::types::TemplateTypeInfo<Mapping,true>(name) {
    RTT::types::newConstructor(&MappingTypeInfo::construct, true);
  }
  virtual ~MappingTypeInfo() {}

  static Mapping construct(const std::string& s) {
    Mapping map;
    map.fromString(s);
    return map;
  }

  virtual bool composeTypeImpl(const RTT::PropertyBag& source, RTT::internal::AssignableDataSource<Mapping>::reference_t result) const {
    // if (source.getType() != RTT::types::TemplateTypeInfo<Mapping,true>::getTypeName()) return false;
    RTT::Property<std::string> *mapString = source.getPropertyType<std::string>("Mapping");
    if (mapString) return result.fromString(mapString->get());
    return false;
  }

  virtual bool decomposeTypeImpl(RTT::internal::AssignableDataSource<Mapping>::const_reference_t source, RTT::PropertyBag& targetbag ) const {
    targetbag.setType(this->getTypeName());
    targetbag.add(new RTT::Property<std::string>("Mapping", "", source.toString()));
    return true;
  }
};

} // namespace uxvcos

#endif // UXVCOS_MAPPINGTYPEINFO_HPP
