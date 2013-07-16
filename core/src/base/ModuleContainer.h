#ifndef UXVCOS_MODULECONTAINER_H
#define UXVCOS_MODULECONTAINER_H

#include <uxvcos.h>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>

#include <vector>
#include <map>

#include <base/Module.h>

namespace uxvcos {

class UXVCOS_EXPORT ModuleContainer
{
public:
  // typedef Module ModuleType;
  typedef boost::shared_ptr<Module> ModulePtr;
  typedef std::vector<ModulePtr> Modules;
  typedef std::map<std::string,ModulePtr> ModuleMap;

  ModuleContainer(RTT::TaskContext *owner = 0) : _owner(owner) {}
  virtual ~ModuleContainer() {}

  const Modules &modules() const { return _modules; }

  template <typename T>
  boost::shared_ptr<T> addModule(T *instance) {
    return addModule(boost::shared_ptr<T>(instance));
  }

  template <typename T>
  boost::shared_ptr<T> addModule(boost::shared_ptr<T> instance) {
    ModulePtr module(boost::shared_static_cast<Module>(instance));
    _modules.push_back(module);
    _module_map[module->getName()] = module;

    if (_owner) _owner->provides()->addService(boost::shared_static_cast<RTT::Service>(module));

    return instance;
  }

  virtual bool initializeModules(bool rewindOnFailure = false) const {
    bool result = true;

    for(typename Modules::const_iterator it = _modules.begin(); it != _modules.end(); ++it) {
      RTT::log( RTT::Debug ) << "  Initializing module " << (*it)->getName() << "..." << RTT::endlog();
      if (!(*it)->initialize()) {
        RTT::log( RTT::Error ) << "    Module " << (*it)->getName() << " failed" << RTT::endlog();
        result = false;
      }

      if (rewindOnFailure && !result) {
        RTT::log( RTT::Error ) << "  rewinding..." << RTT::endlog();
        cleanupModules(typename Modules::const_reverse_iterator(it));
        break;
      }
    }

    return result;
  }

  virtual void executeModules() const {
    for(typename Modules::const_iterator it = _modules.begin(); it != _modules.end(); ++it) {
      (*it)->execute();
    }
  }

  virtual void cleanupModules(typename Modules::const_reverse_iterator it) const {
    for(; it != _modules.rend(); ++it) {
      RTT::log( RTT::Debug ) << "  Cleaning up module " << (*it)->getName() << "..." << RTT::endlog();
      (*it)->cleanup();
    }
  }

  virtual void cleanupModules() const {
    cleanupModules(_modules.rbegin());
  }

  template <typename ModuleType> boost::shared_ptr<ModuleType> getModule(const std::string& name) const
  {
    if (!_module_map.count(name)) return boost::shared_ptr<ModuleType>();
    return boost::shared_dynamic_cast<ModuleType>(_module_map.at(name));
  }

private:
  RTT::TaskContext *_owner;

  Modules _modules;
  ModuleMap _module_map;
};

} // namespace uxvcos

#endif // UXVCOS_MODULECONTAINER_H
