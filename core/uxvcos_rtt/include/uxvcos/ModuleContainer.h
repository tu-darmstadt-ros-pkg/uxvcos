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

#ifndef UXVCOS_MODULECONTAINER_H
#define UXVCOS_MODULECONTAINER_H

#include <uxvcos/uxvcos.h>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>

#include <vector>
#include <map>

#include <uxvcos/Module.h>

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
