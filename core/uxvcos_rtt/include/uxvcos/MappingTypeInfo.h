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
