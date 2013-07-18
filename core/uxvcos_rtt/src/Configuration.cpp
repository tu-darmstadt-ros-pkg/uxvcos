#include "Configuration.h"
#include <rtt/TaskContext.hpp>
#include <rtt/marsh/PropertyLoader.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace uxvcos {

std::list<std::string> Configuration::paths;
std::list<std::string> Configuration::configurations(1, "");
std::string Configuration::extension("cpf");
std::string Configuration::store_path;

class Configuration::Loader {
public:
  Loader(RTT::Service *target) : pl(target) {}

  bool load(const std::string &filename) { return pl.load(filename); }
  bool write(const std::string &filename) { return pl.save(filename, true); }
  bool store(const std::string &filename) { return pl.store(filename); }

  bool updateFrom(const std::string &filename) { return pl.configure(filename, false); }
  bool updateTo(const std::string &filename) { return pl.save(filename, false); }

  bool configure(const std::string &filename, const std::string &name) { return pl.configure(filename, name); }
  bool save(const std::string &filename, const std::string &name) { return pl.save(filename, name); }

private:
  RTT::marsh::PropertyLoader pl;
};

Configuration::Configuration(RTT::TaskContext *owner)
  : service(owner->provides().get())
{
  loader = new Loader(service);
}

Configuration::Configuration(RTT::Service *service)
  : service(service)
{
  loader = new Loader(service);
}

Configuration::~Configuration()
{
  delete loader;
}

void Configuration::setExtension(const std::string& extension) {
  Configuration::extension = extension;
}

void Configuration::addPath(std::string path) {
  // boost::algorithm::to_lower(path);
  if (store_path.empty()) store_path = path;
  paths.push_back(path);
}

void Configuration::addPaths(std::vector<std::string> paths) {
  for(std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it) {
    addPath(*it);
  }
}

void Configuration::addConfiguration(std::string name) {
  // boost::algorithm::to_lower(name);
  configurations.push_back(name);
}

void Configuration::addConfigurations(std::vector<std::string> names) {
  for(std::vector<std::string>::iterator it = names.begin(); it != names.end(); ++it) {
    addConfiguration(*it);
  }
}

std::string Configuration::getPrimaryPath() {
  if (paths.empty()) {
    addPath(".");
  }
  return paths.front();
}

const std::list<std::string>& Configuration::getPaths() {
  return paths;
}

const std::list<std::string>& Configuration::getConfigurations() {
  return configurations;
}

void Configuration::setStorePath(std::string path) {
  store_path = path;
}

std::string Configuration::getStorePath() {
  if (store_path.empty()) return ".";
  return store_path;
}

std::string Configuration::buildFilename(const std::string& path, const std::string& realm, std::string extension) {
  return buildFilename(path, std::string(), realm, extension);
}

std::string Configuration::buildFilename(const std::string& path, const std::string& pathsuffix, const std::string& realm, std::string extension) {
  if (extension.empty()) extension = Configuration::extension;
  return (boost::filesystem::path(path) / pathsuffix / (realm + "." + extension)).string();
}

bool Configuration::read(std::string realm, const std::string& name, bool all) {
  if (realm.empty()) realm = boost::algorithm::to_lower_copy(service->getName());
  if (name.empty()) {
    if (all)
      return for_each(realm, boost::bind(&Loader::load, loader, _1));
    else
      return for_each(realm, boost::bind(&Loader::updateFrom, loader, _1));
  } else {
    return for_each(realm, boost::bind(&Loader::configure, loader, _1, name));
  }
}

bool Configuration::readAll(std::string realm, const std::string& path) {
  return read(realm, path, true);
}


bool Configuration::write(std::string realm, const std::string& name) {
  if (realm.empty()) realm = boost::algorithm::to_lower_copy(service->getName());
  if (name.empty()) {
    return for_each(realm, boost::bind(&Loader::write, loader, _1));
  } else {
    return for_each(realm, boost::bind(&Loader::save, loader, _1, name));
  }
}

bool Configuration::update(std::string realm) {
  if (realm.empty()) realm = boost::algorithm::to_lower_copy(service->getName());
  return for_each(realm, boost::bind(&Loader::updateTo, loader, _1));
}

bool Configuration::store(std::string realm, const std::string& pathsuffix, std::string extension) {
  if (realm.empty()) realm = boost::algorithm::to_lower_copy(service->getName());
  if (extension.empty()) extension = Configuration::extension;

  std::string filename = buildFilename(getStorePath(), pathsuffix, realm, extension);
  return loader->store(filename);
}

bool Configuration::for_each(const std::string& realm, boost::function<bool (const std::string& filename)> operation) {
  bool result = false;

  // ensure that add least one configuration path exists
  getPrimaryPath();

  for(std::list<std::string>::iterator itconfiguration = configurations.begin(); itconfiguration != configurations.end(); ++itconfiguration)
  {
    boost::filesystem::path configuration(*itconfiguration);
    for(std::list<std::string>::iterator itpath = paths.begin(); itpath != paths.end(); ++itpath)
    {
      boost::filesystem::path path(*itpath);
      // if (!path.is_complete()) path = boost::filesystem::path(configurationPath) / path;
      boost::filesystem::path filename = path / configuration / (realm + "." + extension);
      if (boost::filesystem::exists(filename)) result |= operation(filename.string());
    }
  }

  return result;
}

} // namespace uxvcos
