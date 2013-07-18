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
