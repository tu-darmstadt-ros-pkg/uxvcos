#include <system/FileStream.h>
#include <system/systemcalls.h>
#include <options/options.h>
#include <string>
#include <base/SetupFunction.h>

#include <rtt/Activity.hpp>

#include "Logging.h"

using namespace RTT;

namespace uxvcos {
namespace Logging {

static const int priority = 0;

namespace { SetupFunction setup(&Logging::setup, "Logging"); }

Logging::Logging(const std::string& name, DataPool *dataPool)
  : RTT::TaskContext(name)
  , directory("Directory", "Directory in which logfiles will be stored", "")
  , file("Filename", "Name of the logfile", "log")
  , extension("Extension", "File extension of the logfile", "raw")
  , appendTimestamp("AppendTimestamp", "If true, a UTC timestamp in -yyyymmdd-hhmmss format is appended to the filename", true)
  , sync("Sync", "Sync characters in hexadecimal notation")
  , portListener(this, true, RTT::ConnPolicy::buffer(100))
{
  portListener.useDataPool(dataPool);
  this->properties()->addProperty(directory);
  this->properties()->addProperty(file);
  this->properties()->addProperty(extension);
  this->properties()->addProperty(appendTimestamp);
  this->properties()->addProperty(sync);
  construct();
}

Logging::Logging(boost::shared_ptr<OutStream> stream, const std::string& name, DataPool *dataPool)
  : RTT::TaskContext(name)
  , stream(stream)
  , directory("Directory", "Directory in which logfiles will be stored")
  , file("File", "Name of the logfile")
  , extension("Extension", "File extension of the logfile", "raw")
  , appendTimestamp("AppendTimestamp", "If true, a UTC timestamp in -yyyymmdd-hhmmss format is appended to the filename", true)
  , sync("Sync", "Sync characters in hexadecimal notation")
  , portListener(this)
{
  portListener.useDataPool(dataPool);
  this->properties()->addProperty(sync);
  construct();
}

Logging::~Logging() {
  stop();
  cleanup();
}

bool Logging::construct() {
  // this->provides()->addService(RTT::ServicePtr(portListener));
  this->setActivity(new RTT::Activity(priority, 0, this->getName()));
  //this->setActivity(new RTT::extras::SequentialActivity());
  return true;
}

int Logging::setup() {
  Options::Description options("Logging options");
  options.add_options()("log,l", Options::value<std::string>(), "name of the logfile");
  Options::options().add(options);
  return 0;
}

bool Logging::configureHook()
{
  Logger::In in(getName());

  return true;
}

bool Logging::startHook()
{
  Logger::In in(getName());

  if (!Options::variables("log").empty()) {
    fullname = Options::variables("log").as<std::string>();
    file = fullname;
    directory = "";
  }

  if (!stream && fullname.empty()) {
    if (!directory.get().empty()) fullname = directory.get() + "/";
    if (file.get().empty()) file = "log";
    fullname = fullname + file.get();

    if (appendTimestamp.get()) {
      time_t t;
      struct tm * timeinfo;
      char timestamp[18];

      time(&t);
      timeinfo = gmtime(&t);
      snprintf(timestamp, sizeof(timestamp) - 1, "-%04d%02d%02d-%02d%02d%02d",
          timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
          timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
      fullname = fullname + timestamp;
    }
    fullname = fullname + "." + extension.get();
  }

  if (!stream && !fullname.empty()) {
    stream = boost::shared_ptr<OutStream>(new System::OutFileStream(fullname));
  }

  if (!stream || !stream->open()) {
    RTT::log(Error) << "Output stream not ready for logging" << RTT::endlog();
    return false;
  }

  ubx = new UBX::Encoder(stream.get());
  if (!sync.get().empty() && !ubx->ublox()->setSync(sync.get())) {
    RTT::log( RTT::Error ) << "Illegal sync sequence: " << sync.get() << RTT::endlog();
    stopHook();
    return false;
  }

  // listen to all available ports
  portListener.start();
  
  return true;
}

bool Logging::log(RTT::base::DataSourceBase::shared_ptr dsb) {
  if (!(*ubx << dsb)) {
    RTT::log(RTT::RealTime) << "Failed to serialize DataSource of type " << dsb->getType() << RTT::endlog();
    return false;
  }
//  RTT::log( RTT::Debug ) << "Writing " << dsb->getTypeInfo()->getTypeName() << RTT::endlog();
//  dsb->getTypeInfo()->write(std::cout, dsb) << std::endl;
  return true;
}

bool Logging::log(const Data::Streamable& message) {
  if (!(*ubx << message)) {
    RTT::log(RTT::RealTime) << "Failed to serialize object of type " << (message.getTypeInfo() ? message.getTypeInfo()->getTypeName() : "(unknown)") << RTT::endlog();
    return false;
  }
//  RTT::log( RTT::Debug ) << "Writing " << message.getTypeInfo()->getName() << " (t = " << message.getTimestamp() << ")" << RTT::endlog();
//  std::cout << message << std::endl;
  return true;
}

void Logging::updateHook()
{
  // RTT::Logger::In in(getName());

  if (!stream || !stream->open()) {
    stop();
    return;
  }

  const PortListener::InputPorts &ports = portListener.getPorts();
  for(PortListener::InputPorts::const_iterator it = ports.begin(); it != ports.end(); it++) {
    RTT::base::DataSourceBase::shared_ptr dsb = (*it)->getDataSource();
    while(dsb->evaluate()) log(dsb);
  }
  // portListener.free();
}

bool Logging::breakUpdateHook() {
  return true;
}

void Logging::stopHook()
{
  Logger::In in(getName());

  // stop listening
  portListener.stop();

//   ReportingComponent::stopHook();
//   this->removeMarshallers();

  delete ubx;
  if (stream) stream->close();
}

void Logging::cleanupHook()
{
  Logger::In in(getName());
}

} // namespace Logging
} // namespace uxvcos
