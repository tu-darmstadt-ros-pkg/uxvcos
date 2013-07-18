#include <rtt/Activity.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/ThreadInterface.hpp>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <rtt/scripting/ScriptingService.hpp>

#include <signal.h>

#include "uxvcos/version.h"
#include "uxvcos/options/options.h"
#include "Application.h"
#include "Configuration.h"
#include "DataPool.h"

#include <rtt/plugin/PluginLoader.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#ifdef ROS_PACKAGE_NAME
  #include <ros/ros.h>
  #define IS_ROS_NODE
#endif

#define DEFAULT_FREQUENCY 100             //! default frequency of the main loop
// #undef NDEBUG

using namespace RTT;

// workaround for RTT bug #995 (see http://bugs.orocos.org/show_bug.cgi?id=995)
template class RTT::Property<std::string>;

namespace uxvcos {

class Application::ScriptingInterface {
private:
  Application *application;

public:
  ScriptingInterface(Application *application) : application(application) {}

  void setup(TaskContext *tc) {
    tc->provides()->removeOperation("loadComponent");
    tc->addOperation("loadComponent", &ScriptingInterface::loadComponent, this);
    tc->addOperation("addTask", &ScriptingInterface::addTask, this);
    tc->addOperation("addTrigger", &ScriptingInterface::addTrigger, this);
    tc->addOperation("publish", &ScriptingInterface::publish, this);
    tc->addOperation("subscribe", &ScriptingInterface::subscribe, this);
  }

  TaskContext *loadComponent(const std::string& name, const std::string& type, const EchelonType& echelon) {
    return application->loadComponent(name, type, false, echelon);
  }

  bool addTask(TaskContext *task) {
    return application->addTask(task);
  }

  bool addTrigger(TaskContext *task) {
    return application->addTrigger(task);
  }

  bool publish(const std::string& port_name, const ConnPolicy& conn_policy) {
    DataPool::Connection *connection = DataPool::Global()->find(port_name);
    if (connection) {
      if (connection->hasWriters()) {
        return connection->outputPort->createStream(conn_policy);
      } else {
        log(Warning) << "Not publishing " << port_name << " on topic " << conn_policy.name_id << " as the connection has no writers!" << endlog();
      }
    } else {
      return application->deployment()->stream(port_name, conn_policy);
    }
    return false;
  }

  bool subscribe(const std::string& port_name, const ConnPolicy& conn_policy) {
    DataPool::Connection *connection = DataPool::Global()->find(port_name);
    if (connection) {
      if (connection->hasReaders()) {
        return connection->inputPort->createStream(conn_policy);
      } else {
        log(Warning) << "Not subscribing " << port_name << " on topic " << conn_policy.name_id << " as the connection has no readers!" << endlog();
      }
    } else {
      return application->deployment()->stream(port_name, conn_policy);
    }
    return false;
  }
};

Application* Application::instance = 0;
int Application::exit_code = 0;

Application::Application(const std::string& name)
  : name(name)
  , realm(boost::algorithm::to_lower_copy(name))
  , scriptingInterface(new ScriptingInterface(this))
{
  if (!instance) instance = this;

  applicationEchelon = Echelon::None;
  _abortOnErrors = false;
  signal = 0;
  argc = 0;
  argv = 0;

  // UXVCOS_LOAD_TYPEKIT(uxvcos);

  // OCL::ComponentLoader::Instance()->setComponentPath(plugin::PluginLoader::Instance()->getPluginPath());
  // OCL::ComponentLoader::Instance()->import(plugin::PluginLoader::Instance()->getPluginPath());

  Options::add_options()
    ("frequency", Options::value(&frequency)->default_value(DEFAULT_FREQUENCY), "Update frequency of the main loop");

  Options::add_options()
    ("config-dir", Options::value(&configurationPaths)->multitoken(), "Configuration directory")
    ("config", Options::value(&configurations)->multitoken(), "Additional configuration subdirectories to be searched (relative to the main configuration directory)")
    ("realm", Options::value(&realm), "Application realm (default name for configuration directories, script files, ...)")
    ("log-level", Options::value<std::string>()->default_value("Info"), "The desired log-level of the application ranging from 0 (Never) to 7 (RealTime)")
    ("debug", "Enables debug output and sets the log-level to RealTime")
    ("version", "Print version information.")
    ("help", "This help.");
}

Application::~Application()
{
  Logger::In in(getName());

  if (root()->isRunning()) root()->stop();
  if (root()->isConfigured()) root()->cleanup();

  Tasks copy = tasks;
  for(Tasks::reverse_iterator it = copy.rbegin(); it != copy.rend(); it++) {
    this->removeTask(it->get<0>());
    delete it->get<0>();
  }
  // tasks.clear();
}

void Application::setProgramOptions(int &argc, char **&argv)
{
  this->argc = &argc;
  this->argv = &argv;
}

bool Application::import(const std::string &package)
{
  return deployment()->import(package);
}

void Application::path(const std::string &path)
{
  deployment()->path(path);
}

TaskContext *Application::loadComponent(const std::string& name, const std::string& type, bool required, EchelonType echelon) {
  if (applicationEchelon && echelon && echelon <= applicationEchelon) return 0;

  TaskContext *task = 0;
  if (deployment()->loadComponent(name, type)) task = deployment()->myGetPeer(name);

  if (required && !task) {
    throw std::runtime_error("Component " + name + " of type " + type + " is required but could not be instantiated, terminating...");
  }

  echelons[task] = echelon;
  return task;
}

TaskContext* Application::addTask(TaskContext* task, base::ActivityInterface *activity, EchelonType echelon) {
  if (!task) { delete activity; return task; }

  // prevent double insertions
  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (it->get<0>() == task) { delete activity; return task; }
  }

  if (!echelons[task]) echelons[task] = echelon; else echelon = echelons[task];
  tasks.push_back(boost::tuples::make_tuple(task, boost::bind(&TaskContext::update, task)));
  root()->addPeer(task);
  DataPool::Global()->addComponent(task);

  if (echelon) {
    if (root()->provides("ExecutionTimer")) {
      timers[task] = new Attribute<double>(task->getName());
      root()->provides("ExecutionTimer")->addAttribute(*timers[task]);
    }

    // if (!activity) activity = new extras::SlaveActivity(this->getPeriod());
    if (!activity) activity = new extras::SlaveActivity(this->getMainActivity());
  }

  if (activity) task->setActivity(activity);
  return task;
}

TaskContext* Application::addTrigger(TaskContext *task, EchelonType echelon) {
  if (!task) return task;

  // prevent double insertions
  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (it->get<0>() == task) return task;
  }

  if (!echelons[task]) echelons[task] = echelon;
  tasks.push_back(boost::tuples::make_tuple(task, boost::bind(&TaskContext::trigger, task)));
  root()->addPeer(task);
  DataPool::Global()->addComponent(task);

  return task;
}

void Application::removeTask(TaskContext* task) {
  if (!task) return;

  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (it->get<0>() == task) {
      task->stop();
      task->cleanup();
      if (timers[task]) {
        root()->provides("ExecutionTimer")->removeAttribute(task->getName());
        delete timers[task];
        timers.erase(task);
      }
      tasks.erase(it);
      echelons.erase(task);
      break;
    }
  }
}

const Application::Tasks& Application::getTasks() const {
  return tasks;
}

TaskContext* Application::getTask(const std::string& name) const {
  for(Tasks::const_iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (it->get<0>()->getName() == name) return it->get<0>();
  }
  return 0;
}

OCL::TaskBrowser *Application::browser() {
  if (!_browser && root()) {
    // initialize Task Browser
    #ifdef ORO_TASKBROWSER_HPP
      #ifdef SIGWINCH
        ::signal(SIGWINCH, SIG_IGN);
      #endif
      _browser.reset(new OCL::TaskBrowser(root()));
    #endif
  }
  return _browser.get();
}

OCL::DeploymentComponent *Application::deployment() {
  return _deployment.get();
}

TaskContext *Application::root() {
  return _deployment.get();
}

boost::shared_ptr<scripting::ScriptingService> Application::scripting() {
  if (!_scripting && root()) {
    root()->loadService("scripting");
    _scripting = boost::shared_dynamic_cast<scripting::ScriptingService>(root()->provides("scripting"));

    if (!_scripting) {
      log(Error) << "Could not load scripting service" << endlog();
    }
  }
  return _scripting;
}

bool Application::setup() {
  Logger::In in(getName());

  // start ROS node
#ifdef IS_ROS_NODE
  if (!ros::isInitialized()) {
    ros::master::setRetryTimeout(ros::WallDuration(5.0));
    ros::init(*argc, *argv, realm);
  }
#endif

  // parse command line options (stage 1)
  if (!argc || !argv) return false;
  Options::parse(*argc, *argv);

  // set log level
  if (!Options::variables("log-level").empty()) {
    Logger::LogLevel loglevel = Logger::Never;
    std::string s = boost::algorithm::to_lower_copy(Options::variables<std::string>("log-level"));

    if (s == "0" || s == "never")         loglevel = Logger::Never;
    else if (s == "1" || s == "fatal")    loglevel = Logger::Fatal;
    else if (s == "2" || s == "critical") loglevel = Logger::Critical;
    else if (s == "3" || s == "error")    loglevel = Logger::Error;
    else if (s == "4" || s == "warning")  loglevel = Logger::Warning;
    else if (s == "5" || s == "info")     loglevel = Logger::Info;
    else if (s == "6" || s == "debug")    loglevel = Logger::Debug;
    else if (s == "7" || s == "realtime") {
      Logger::Instance()->allowRealTime();
      loglevel = Logger::RealTime;
    } else {
      std::cerr << "--log-level has to be numeric or one of the values Never, Fatal, Critical, Error, Warning, Info, Debug or RealTime" << std::endl;
      exit(0);
      return false;
    }

    Logger::Instance()->setLogLevel(loglevel);
  }

  if (!Options::variables("debug").empty()) {
    Logger::Instance()->allowRealTime();
    Logger::Instance()->setLogLevel(Logger::RealTime);
  }

  // print version
  log(Info) << "This is " << getProjectName() << " version " << getVersionString() << endlog();
  if (!Options::variables("version").empty()) {
    exit(0);
    return false;
  }

  // initialize deployment
  _deployment.reset(new OCL::DeploymentComponent(name));
  _deployment->properties()->getPropertyType<bool>("AutoUnload")->set(false);
  scriptingInterface->setup(_deployment.get());

  root()->addConstant("realm", realm);
  root()->addAttribute("level", applicationEchelon);

  root()->provides()->addOperation("configureTasks", &Application::configureTasks, this);
  root()->provides()->addOperation("connectTasks",   &Application::connectTasks,   this);
  root()->provides()->addOperation("startTasks",     &Application::startTasks,     this);
  root()->provides()->addOperation("stopTasks",      &Application::stopTasks,      this);
  root()->provides()->addOperation("cleanupTasks",   &Application::cleanupTasks,   this);
  root()->provides()->addOperation("addConfigurationPath", &Configuration::addPath);
  root()->provides()->addOperation("addConfiguration", &Configuration::addConfiguration);

  root()->provides()->addOperation("getOption", &Application::getOptionDataSource, this);

  root()->addPeer(DataPool::Global());

  root()->addEventPort("trigger", triggerPort, boost::bind(&Application::triggerCallback, this, _1));

  Service::Create("ExecutionTimer", root());

  // set Activity of application
  if (frequency > 0.0) {
    this->setMainActivity(new Activity(os::HighestPriority, 1.0 / frequency, 0, getName()));
  } else {
    this->setMainActivity(new Activity(os::HighestPriority, 0, getName()));
  }

  // add command line options as attributes
  for(OptionAliases::iterator it = optionAliases.begin(); it != optionAliases.end(); ++it) {
    Alias alias(it->first, it->second);
    if (!root()->getAttribute(alias.getName())) root()->addAttribute(alias);
  }

  // setup application (load all components)
  if (!setupHook()) return false;

  // parse command line options (stage 2)
  if (!Options::parse(*argc, *argv)) {
    // Options::help();
    // return false;
  }

  // display help and exit...
  if (!Options::variables("help").empty()) {
    Options::help();
    return false;
  }

  return true;
}

bool Application::configure() {
  Logger::In in(getName());
  log(Info) << "Configuring application..." << endlog();

  // set configurationPath(s)
  Configuration::addPaths(configurationPaths);
  Configuration::addConfigurations(configurations);

  if (!configureHook()) return false;
  if (!configureTasks()) return false;
  // if (!connectTasks()) return false;

  log(Info) << "Configuration finished." << endlog();
  return true;
}

bool Application::start() {
  Logger::In in(getName());
  log(Info) << "Starting application..." << endlog();

  if (!startHook()) return false;
  if (!startTasks()) return false;
  root()->start();
  runningHook();

  log(Info) << "Running." << endlog();
  return true;
}

void Application::stop() {
  Logger::In in(getName());
  log(Info) << "Stopping application..." << endlog();

  if (!beforeStopHook()) return;
  root()->stop();
  stopTasks();
  stopHook();

  log(Info) << "Stopped." << endlog();
}

void Application::cleanup() {
  Logger::In in(getName());
  log(Info) << "Cleaning up..." << endlog();

  cleanupTasks();
  cleanupHook();

  log(Info) << "Application finished." << endlog();
}

void Application::wait()
{
  Logger::In in(getName());

#ifdef ORO_TASKBROWSER_HPP
  if (browser()) {
    browser()->loop();
  }
#else
  pause();
#endif
}

bool Application::stopTasks()
{
  for(Tasks::reverse_iterator it = tasks.rbegin(); it != tasks.rend(); it++) {
    if (it->get<0>()->isRunning()) it->get<0>()->stop();
  }

  return true;
}

bool Application::cleanupTasks()
{
  for(Tasks::reverse_iterator it = tasks.rbegin(); it != tasks.rend(); it++) {
    if (it->get<0>()->isConfigured()) it->get<0>()->cleanup();
  }

  return true;
}

void Application::abort() {
  raise(SIGINT);
}

int Application::exit(int status) {
  if (status != 0) exit_code = status;
  ::exit(exit_code);
  return exit_code;
}

bool Application::setupHook() { return true; }
bool Application::configureHook() { return true; }
bool Application::startHook() { return true; }
void Application::runningHook() { }
bool Application::beforeStopHook() { return true; }
void Application::stopHook() { }
void Application::cleanupHook() { }

bool Application::initialize() {
#ifndef NDEBUG
  log( RealTime ) << __FUNCTION__ << endlog();
#endif
  return true;
}

void Application::step() {
#ifndef NDEBUG
  log( RealTime ) << __FUNCTION__ << endlog();
#endif

  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); ++it) {
    TaskContext *task = it->get<0>();
    boost::function<bool(void)> func = it->get<1>();
    EchelonType echelon = echelons[task];

    if (!echelon) continue;
    if (applicationEchelon && echelon <= applicationEchelon) continue;
    uxvcos::Time start = uxvcos::Time::now();

#ifndef NDEBUG
    log( RealTime ) << "Executing/Triggering " << task->getName() << endlog();
#endif

    func();

    if (timers[task]) timers[task]->set((uxvcos::Time::now() - start).toSec());
  }
}

bool Application::breakLoop() {
  return true;
}

void Application::finalize()
{
#ifndef NDEBUG
  log( RealTime ) << __FUNCTION__ << endlog();
#endif
}

bool Application::setMainActivity(base::ActivityInterface* new_act)
{
  if (main_activity) main_activity->stop();

  main_activity.reset(new_act);

  if (main_activity) {
    main_activity->stop();
    main_activity->run(this);
    main_activity->thread()->setWaitPeriodPolicy(ORO_WAIT_ABS);
    return main_activity->start();
  }

  return true;
}

base::ActivityInterface* Application::getMainActivity()
{
  return main_activity.get();
}

bool Application::execute()
{
  if (!main_activity) return false;
  return main_activity->execute();
}

bool Application::trigger()
{
  if (!main_activity) return false;
  return main_activity->trigger();
}

bool Application::configureTasks()
{
  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (!it->get<0>()->isConfigured() && !it->get<0>()->configure()) {
      log( Error ) << "failed to configure " << it->get<0>()->getName() << endlog();
      if (_abortOnErrors) return false;
    }
  }

  return true;
}

bool Application::connectTasks()
{
  Logger::In in(getName());

  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    DataPool::Global()->addComponent(it->get<0>());
  }

  return true;
}

bool Application::startTasks()
{
  for(Tasks::iterator it = tasks.begin(); it != tasks.end(); it++) {
    if (it->get<0>()->getActivity() && !it->get<0>()->isRunning() && !it->get<0>()->start()) {
      log( Error ) << "failed to start " << it->get<0>()->getName() << endlog();
      if (_abortOnErrors) return false;
    }
  }

  return true;
}

// catch ctrl+c signal
void Application::signal_handler(int sig)
{
  const char * signame;

  switch(sig) {
    case SIGINT: signame = "SIGINT"; break;
    case SIGTERM: signame = "SIGTERM"; break;
    default: signame = "a"; break;
  }

  log( Info ) << "got " << signame << " signal..." << endlog();
  Application::Instance()->signal = sig;
}

void Application::pause()
{
  ::signal(SIGINT, Application::signal_handler);
  ::signal(SIGTERM, Application::signal_handler);

  while(signal != SIGINT) {
    usleep(1000);
  }

  ::signal(SIGINT, SIG_DFL);
  ::signal(SIGTERM, SIG_DFL);
}

RTT::base::DataSourceBase::shared_ptr Application::getOptionDataSource(const std::string& name)
{
  const boost::program_options::variable_value &value = Options::variables(name);
  if (value.empty()) return RTT::base::DataSourceBase::shared_ptr();

  try {
    return RTT::base::DataSourceBase::shared_ptr(new RTT::internal::ValueDataSource<std::string>(value.as<std::string>()));
  } catch(boost::bad_any_cast& e) {}

  try {
    return RTT::base::DataSourceBase::shared_ptr(new RTT::internal::ValueDataSource<bool>(value.as<bool>()));
  } catch(boost::bad_any_cast& e) {}

  try {
    return RTT::base::DataSourceBase::shared_ptr(new RTT::internal::ValueDataSource<int>(value.as<int>()));
  } catch(boost::bad_any_cast& e) {}

  RTT::log(RTT::Error) << "Option " << name << " has an unknown type" << RTT::endlog();
  return RTT::base::DataSourceBase::shared_ptr();
}

void Application::triggerCallback(RTT::base::PortInterface *port) {
  this->trigger();
}

} // namespace uxvcos
