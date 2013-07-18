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

#ifndef UXVCOS_CHILD_H
#define UXVCOS_CHILD_H

#include <uxvcos/uxvcos.h>
#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/typekit/RTTTypes.hpp>

#include <string>

namespace uxvcos {
class UXVCOS_EXPORT Child : public RTT::Property<RTT::PropertyBag> {
public:
  Child(const std::string& name, const std::string& description = "")
    :  RTT::Property<RTT::PropertyBag>(name, description)
  {}

  Child(RTT::PropertyBag *bag, const std::string& name, const std::string& description = "")
    :  RTT::Property<RTT::PropertyBag>(name, description, new RTT::internal::ReferenceDataSource<RTT::PropertyBag>(*bag))
  {}

  virtual ~Child() { }

  RTT::PropertyBag* properties() { return &(this->value()); }
  const RTT::PropertyBag* properties() const { return &(this->rvalue()); }

  template <typename T>
  RTT::Property<T>& addProperty(const std::string &name, const std::string &description, T& ref) {
    return properties()->addProperty(name, ref).doc(description);
  }

  template <typename T>
  void declareProperty(RTT::Property<T> &prop) {
    RTT::base::PropertyBase *other = properties()->getProperty(prop.getName());
    if (other) {
      prop = other;
    } else {
      properties()->addProperty(prop);
    }
  }

  template <typename T>
  RTT::Property<T>& declareProperty(const std::string &name, const std::string &description, T& ref) {
    RTT::Property<T> *prop = properties()->getPropertyType<T>(name);
    if (prop) return *prop;
    return addProperty(name, description, ref);
  }

//  std::string getName() const { return prop.getName(); }
//  Child& setName(const std::string& name) { prop.setName(name); return *this; }
//  std::string getDescription() const { return prop.getDescription(); }
//  Child& setDescription(const std::string& description) { prop.setDescription(description); return *this; }
  std::string getType() const { return this->rvalue().getType(); }
  Child& setType(const std::string& type) { this->value().setType(type); return *this; }
};
} // namespace uxvcos

#endif
