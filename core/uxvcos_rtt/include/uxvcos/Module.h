#ifndef UXVCOS_MODULE_H
#define UXVCOS_MODULE_H

#include <uxvcos/uxvcos.h>
#include <rtt/TaskContext.hpp>
#include <uxvcos/Child.h>

namespace uxvcos {

class ModuleContainer;

class UXVCOS_EXPORT Module : public RTT::Service, public Child {
public:
  typedef boost::shared_ptr<Module> shared_ptr;

  Module(RTT::TaskContext* parent, const std::string& name, const std::string& description = "");
  virtual ~Module();

  shared_ptr module() { return boost::shared_static_cast<Module>(this->shared_from_this()); }

  virtual bool initialize();
  virtual void execute();
  virtual void cleanup();

  using RTT::base::PropertyBase::getName;
  using RTT::base::PropertyBase::setName;

  using RTT::base::PropertyBase::getDescription;
  using RTT::base::PropertyBase::setDescription;

  using RTT::ConfigurationInterface::properties;
  using RTT::ConfigurationInterface::addProperty;
};

} // namespace uxvcos

#endif
