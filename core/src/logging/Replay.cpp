#include <system/FileStream.h>
#include <system/Error.h>
#include <string>
#include <options/options.h>
#include <base/SetupFunction.h>

#include <rtt/Activity.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/fosi.h>

#include <base/Application.h>
#include "Replay.h"

using namespace RTT;

namespace uxvcos {
namespace Logging {

namespace { SetupFunction setup(&Replay::setup, "Replay"); }

Replay::Replay(const std::string& name)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  , directory("Directory", "Directory in which logfiles are stored", "./log")
  , file("Filename", "Name of the logfile", "log.raw")
  , rate("Rate")
  , startTime("Start")
  , stopTime("Stop")
  , dataPool(DataPool::Global())
{
  this->properties()->addProperty(directory);
  this->properties()->addProperty(file);
  construct();
}

Replay::Replay(boost::shared_ptr<InStream> stream, const std::string& name)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  , stream(stream)
  , directory("Directory", "Directory in which logfiles are stored")
  , file("File", "Name of the logfile")
  , rate("Rate")
  , startTime("Start")
  , stopTime("Stop")
  , dataPool(DataPool::Global())
{
  construct();
}

Replay::~Replay() {
  stop();
  cleanup();
}

int Replay::setup() {
  Options::Description options("Replay options");
  options.add_options()
      ("replay,r", Options::value<std::string>(), "name of the logfile")
//      ("level", Options::value<unsigned>()->default_value(Sensors),
//          "replay level:\n  1 = Raw\n  2 = Sensors (default)\n  3 = Navigation\n  4 = Controller\n  5 = Logfile\n")
      ("rate", Options::value<double>(), "time factor (empty = as fast as possible)")
      ("start", Options::value<double>(), "start time in seconds")
      ("stop", Options::value<double>(), "stop time in seconds");
  Options::options().add(options);

  return 0;
}

bool Replay::construct() {
  this->provides()->addAttribute(rate);
  this->provides()->addAttribute(startTime);
  this->provides()->addAttribute(stopTime);

  _mode = Echelon::None;

  if (!Options::variables("replay").empty()) {
    directory.set("");
    file.set(Options::variables("replay").as<std::string>());
  } else {
    return false;
  }

  if (!Options::variables("level").empty()) {
    _mode = static_cast<Mode>(Options::variables("level").as<unsigned>());
  }

  if (!Options::variables("rate").empty()) {
    rate.set(Options::variables("rate").as<double>());
  }

  if (!Options::variables("start").empty()) {
    startTime.set(Options::variables("start").as<double>());
  }

  if (!Options::variables("stop").empty()) {
    stopTime.set(Options::variables("stop").as<double>());
  }

  // _mode = Replay::Sensors;
  return true;
}

bool Replay::configureHook()
{
  Logger::In in(getName());

  if (!mode()) return false;

  if (!stream) {
    std::string fullname = directory.get();
    if (!fullname.empty()) fullname += "/";
    fullname += file.get();
    stream.reset(new System::InFileStream(fullname));
  }

  if (!stream || !stream->open()) {
    log(Error) << "Input stream not ready: " << System::Error::lastError() << endlog();
    return false;
  }

  decoder = new UBX::Decoder(stream.get());
  timestamp = uxvcos::Time();

  // make the main activity a slave activity
  Application::Instance()->setMainActivity(new RTT::extras::SlaveActivity());

  return true;
}

bool Replay::startHook()
{
  Logger::In in(getName());

  realTimestamp = uxvcos::Time::now();
  progress = uxvcos::Time();
  return true;
}

void Replay::updateHook()
{
  // Logger::In in(getName());

  Data::Streamable* data = 0;
  uxvcos::Time realTime;
  
  if (!stream) return;

  breakUpdate = false;
  while(!breakUpdate && !stream->eof()) {
    data = decoder->read();
    if (!Data::Streamable::isValid(data)) continue;
    
    if (decoder->getTimestamp() != timestamp) {
      uxvcos::Time now = uxvcos::Time::now();
      Application::Instance()->execute();

      if (!timestamp.isZero() && rate.get() > 0) {
        uxvcos::Duration pause = (realTimestamp + uxvcos::Duration((decoder->getTimestamp() - timestamp).toSec() / rate.get())) - now;
        if (pause > uxvcos::Duration(0)) usleep(static_cast<unsigned int>(pause.toSec() * 1e6));
      }

      realTimestamp = now;
      timestamp = decoder->getTimestamp();

      if (timestamp - progress > uxvcos::Duration(10.0)) {
        RTT::log(RTT::Info) << "Replaying... t = " << timestamp << RTT::endlog();
        progress = timestamp;
      }
    }

    if (startTime.get() > 0 && timestamp.toSec() < startTime.get()) continue;
    if (stopTime.get() > 0  && timestamp.toSec() > stopTime.get()) continue;

    if (!dataPool) continue;
    dataPool->inject(data);
  }
  
  if (stream->eof()) {
    Application::Instance()->execute();
    stop();

    RTT::log( RTT::Info ) << "Replay finished." << RTT::endlog();
    Application::Instance()->abort();
  }
}

bool Replay::breakUpdateHook() {
  breakUpdate = true;
  return true;
}

void Replay::stopHook()
{
  Logger::In in(getName());
}

void Replay::cleanupHook()
{
  Logger::In in(getName());

  if (decoder) {
    delete decoder;
    decoder = 0;
  }

  if (stream) {
    stream->close();
    stream.reset();
  }
}

} // namespace Logging
} // namespace uxvcos
