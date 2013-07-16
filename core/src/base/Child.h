#ifndef UXVCOS_CHILD_H
#define UXVCOS_CHILD_H

#include <uxvcos.h>
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
