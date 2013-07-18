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

#ifndef UXVCOS_CONFIGURATION_H
#define UXVCOS_CONFIGURATION_H

#include <uxvcos/uxvcos.h>
#include <rtt/Service.hpp>

#include <list>
#include <vector>
#include <boost/function.hpp>

namespace uxvcos {

class UXVCOS_API Configuration
{
private:
  RTT::Service* service;
  class Loader;
  Loader *loader;

  static std::list<std::string> paths;
  static std::string store_path;
  static std::list<std::string> configurations;
  static std::string extension;

public:
  Configuration(RTT::TaskContext *owner);
  Configuration(RTT::Service *service);
  virtual ~Configuration();

  static void setExtension(const std::string& extension);
  static void addPath(std::string path);
  static void addPaths(std::vector<std::string> paths);
  static void addConfiguration(std::string name);
  static void addConfigurations(std::vector<std::string> names);
  static std::string getPrimaryPath();
  static const std::list<std::string>& getPaths();
  static const std::list<std::string>& getConfigurations();

  static void setStorePath(std::string path);
  static std::string getStorePath();

  static std::string buildFilename(const std::string& path, const std::string& realm, std::string extension = std::string());
  static std::string buildFilename(const std::string& path, const std::string& realm, const std::string& pathsuffix, std::string extension);

  bool read(std::string realm = std::string(), const std::string& path = std::string(), bool all = false);
  bool readAll(std::string realm = std::string(), const std::string& path = std::string());
  bool write(std::string realm = std::string(), const std::string& path = std::string());
  bool update(std::string realm = std::string());
  bool store(std::string realm = std::string(), const std::string& pathsuffix = std::string(), std::string extension = std::string());

private:
  bool for_each(const std::string& realm, boost::function<bool (const std::string& filename)> operation);
  bool for_each(const std::string& realm, const std::string& name, boost::function<bool (const std::string& name,const std::string& filename)> operation);
};

} // namespace uxvcos

#endif // UXVCOS_CONFIGURATION_H
