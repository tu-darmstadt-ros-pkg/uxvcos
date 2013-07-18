#ifndef UXVCOS_APPLICATION_H
#define UXVCOS_APPLICATION_H

#include "uxvcos/uxvcos.h"
#include "uxvcos/options/options.h"

#include <rtt/TaskContext.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/Logger.hpp>

#include <vector>
#include <boost/tuple/tuple.hpp>
#include <boost/function.hpp>

#include <rtt/InputPort.hpp>
#include <rosgraph_msgs/Clock.h>

#include "Echelon.h"

#include <rtt/scripting/rtt-scripting-fwd.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/DeploymentComponent.hpp>


namespace uxvcos {

class UXVCOS_API Application : public RTT::base::RunnableInterface {
  public:
    typedef boost::tuple<RTT::TaskContext *, boost::function<bool(void)> > Task;
    typedef std::vector<Task> Tasks;

    Application(const std::string& name = "Main");
    virtual ~Application();

    static Application* Instance() { return instance; }

    const std::string& getName() const { return name; }
    void setName(const std::string& name) { this->name = name; }

    const std::string& getRealm() const { return realm; }
    void setRealm(const std::string& name) { this->realm = name; }

    void setEchelon(EchelonType echelon) { applicationEchelon = echelon; }
    EchelonType getEchelon() const { return applicationEchelon; }

    bool setMainActivity(RTT::base::ActivityInterface* new_act);
    RTT::base::ActivityInterface* getMainActivity();

    void setProgramOptions(int &argc, char **&argv);

    bool setup();
    bool configure();
    bool start();
    bool execute();
    bool trigger();
    void stop();
    void cleanup();

    void pause();
    void wait();
    void abort();
    static int exit(int code = 0);

    bool import(const std::string &package);
    void path(const std::string &path);

    RTT::TaskContext *loadComponent(const std::string& name, const std::string& type, bool required = false, EchelonType echelon = Echelon::None);
    RTT::TaskContext* addTask(RTT::TaskContext* task, RTT::base::ActivityInterface *activity = 0, EchelonType echelon = Echelon::None);
    RTT::TaskContext* addTrigger(RTT::TaskContext* task, EchelonType echelon = Echelon::Default);
    void removeTask(RTT::TaskContext* task);

    template <typename T> bool addOption(const std::string& name, const std::string& description = std::string());
    template <typename T> bool addOption(const std::string& name, T& value, const std::string& description = std::string());
    template <typename T> bool addOption(const std::string& name, RTT::base::DataSourceBase::shared_ptr dsb, const std::string& description = std::string());
    bool hasOption(const std::string& name);
    template <typename T> const T& getOption(const std::string& name);
    RTT::base::DataSourceBase::shared_ptr getOptionDataSource(const std::string& name);

    const Tasks& getTasks() const;
    RTT::TaskContext* getTask(const std::string& name) const;

    OCL::TaskBrowser *browser();
    OCL::DeploymentComponent *deployment();
    RTT::TaskContext *root();
    boost::shared_ptr<RTT::scripting::ScriptingService> scripting();

    void abortOnErrors(bool flag = true) { _abortOnErrors = flag; }

  protected:
    virtual bool setupHook();
    virtual bool configureHook();
    virtual bool startHook();
    virtual void runningHook();
    virtual bool beforeStopHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:
    virtual bool initialize();
    virtual void step();
    virtual bool breakLoop();
    virtual void finalize();

    bool configureTasks();
    bool connectTasks();
    bool startTasks();
    bool stopTasks();
    bool cleanupTasks();

  private:
    Tasks tasks;
    std::map<RTT::TaskContext *, EchelonType> echelons;
    std::map<RTT::TaskContext *, RTT::Attribute<double> *> timers;

    std::string name;
    std::string realm;
    EchelonType applicationEchelon;

    std::vector<std::string> configurationPaths;
    std::vector<std::string> configurations;
    double frequency;

    bool _abortOnErrors;

    boost::shared_ptr<OCL::TaskBrowser> _browser;
    boost::shared_ptr<OCL::DeploymentComponent> _deployment;
    boost::shared_ptr<RTT::scripting::ScriptingService> _scripting;

    int signal;
    static int exit_code;
    static void signal_handler(int sig);

    static Application* instance;
    boost::shared_ptr<RTT::base::ActivityInterface> main_activity;

    int *argc;
    char ***argv;

    typedef std::map<std::string,RTT::base::DataSourceBase::shared_ptr> OptionAliases;
    OptionAliases optionAliases;

    class ScriptingInterface;
    boost::shared_ptr<ScriptingInterface> scriptingInterface;

    RTT::InputPort<rosgraph_msgs::Clock> triggerPort;
    void triggerCallback(RTT::base::PortInterface *port);
};

template <typename T>
inline bool Application::addOption(const std::string& name, const std::string& description) {
  return addOption<T>(name, new RTT::internal::ValueDataSource<T>(), description);
}

template <typename T>
inline bool Application::addOption(const std::string& name, T& value, const std::string& description)
{
  return addOption<T>(name, new RTT::internal::ReferenceDataSource<T>(value), description);
}

template <typename T>
inline bool Application::addOption(const std::string& name, RTT::base::DataSourceBase::shared_ptr dsb, const std::string& description)
{
  typename RTT::internal::AssignableDataSource<T>::shared_ptr data = RTT::internal::AssignableDataSource<T>::narrow(dsb.get());
  if (!data) return false;
  optionAliases[name] = data;

  Options::add_options()(name.c_str(), Options::value<T>(&(data->set())), description.c_str());
  return true;
}

template <>
inline bool Application::addOption<bool>(const std::string& name, RTT::base::DataSourceBase::shared_ptr dsb, const std::string& description)
{
  typename RTT::internal::AssignableDataSource<bool>::shared_ptr data = RTT::internal::AssignableDataSource<bool>::narrow(dsb.get());
  if (!data) return false;
  optionAliases[name] = data;

  Options::add_options()(name.c_str(), Options::value<bool>(&(data->set()))->zero_tokens()->implicit_value(true), description.c_str());
  return true;
}

template <typename T>
inline const T& Application::getOption(const std::string& name)
{
  return Options::variables(name).as<T>();
}

inline bool Application::hasOption(const std::string& name)
{
  return !Options::variables(name).empty();
}

} // namespace uxvcos

#ifndef APPLICATION
  #define APPLICATION DefaultApplication
  #define APPLICATION_NAME "uxvcos"
#endif

#ifndef APPLICATION_NAME
  #define _xstringify(x) _stringify(x)
  #define _stringify(x) #x
  #define APPLICATION_NAME _xstringify(APPLICATION)
#endif

#endif // UXVCOS_APPLICATION_H
