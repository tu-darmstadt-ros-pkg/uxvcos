//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include <ublox_gps/gps.h>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Component.hpp>

#include <system/BaseSerialPort.h>

#include <sensor_msgs/typekit/NavSatFix.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

namespace uxvcos {
namespace ublox_gps {

using namespace ::ublox_gps;

class GPS : public RTT::TaskContext
{
public:
  GPS(const std::string& name = "GPS", const std::string& description = "Interface to a uBlox GPS receiver");
  ~GPS();

  void setSerialPort(System::BaseSerialPort *port);
  std::string getDevice() const;

  bool configureDynModel(unsigned int dynModel);
  bool configureFixMode(unsigned int fixMode);

protected:
  virtual bool configureHook();
  virtual void updateHook();
  virtual void cleanupHook();

  void handleNavPOSLLH(const ublox_msgs::NavPOSLLH&);
  void handleNavVELNED(const ublox_msgs::NavVELNED&);
  void handleNavSTATUS(const ublox_msgs::NavSTATUS&);

  RTT::OutputPort<sensor_msgs::NavSatFix> portFix;
  RTT::OutputPort<geometry_msgs::Vector3Stamped> portVelocity;

  sensor_msgs::NavSatFix fix;
  geometry_msgs::Vector3Stamped velocity;
  ublox_msgs::NavSTATUS status;

private:
  std::string device;
  std::string frame_id;
  int retries;
  System::BaseSerialPort *port;
  Gps gps;

private:
  struct Worker : public ::ublox_gps::Worker, RTT::base::RunnableInterface {
    Worker(System::BaseSerialPort *port);
    ~Worker();

    void setCallback(const Callback& callback) { this->callback_ = callback; }
    bool send(const unsigned char *data, const unsigned int size);
    void wait(const boost::posix_time::time_duration& timeout);

    bool initialize();
    void step();
    void loop();
    bool breakLoop();
    void finalize();

  private:
    RTT::Activity activity_;
    System::BaseSerialPort *port_;

    boost::array<unsigned char,1024> buffer_;
    std::size_t buffer_length_;

    Worker::Callback callback_;
    bool break_worker_;
  };
};

GPS::GPS(const std::string& name, const std::string& description)
  : RTT::TaskContext(name, RTT::TaskContext::PreOperational)
  , port(0)
{
  this->addPort("fix", portFix);
  this->addPort("fix_velocity", portVelocity);
  this->addProperty("device", device).doc("Name of the SerialPort device to use for GPS communication");
  this->addProperty("frame_id", frame_id).doc("frame_id to be sent in fix and fix_velocity headers");

  retries = 3;
  this->addProperty("retries", retries).doc("Number of retries during the configuration procedure");

  this->addOperation("configureDynModel", &GPS::configureDynModel, this, RTT::OwnThread);
  this->addOperation("configureFixMode", &GPS::configureFixMode, this, RTT::OwnThread);
}

GPS::~GPS()
{
  stop();
  cleanup();
}

void GPS::setSerialPort(System::BaseSerialPort *port) {
  this->port = port;
}

std::string GPS::getDevice() const {
  return device;
}

bool GPS::configureHook() {
  RTT::Logger::In in(getName());
  int retries = this->retries;
  bool success = false;

  // set GPS port
  if (!port) {
    port = System::getSerialPort(device);
    if (!port) return false;
  }
  RTT::log( RTT::Info ) << "Using port " << device << " for the GPS" << RTT::endlog();

  do {
    // initialize GPS and create Worker activity
    gps.initialize(boost::shared_ptr< Worker >(new Worker(port)));

    // configure baud rate
    unsigned long baudrate[] = { 9600, 38400, 57600 };
    bool baudrate_configured = false;
    for (int i = 0; !baudrate_configured && i < 3; i++) {
      port->close();
      if (!port->setBaudrate(baudrate[i])) {
        RTT::log(RTT::Error) << "Could not set GPS port baudrate" << RTT::endlog();
        gps.close();
        continue;
      }
      if (!port->open()) {
        RTT::log(RTT::Error) << "Could not open GPS port" << RTT::endlog();
        gps.close();
        continue;
      }
      baudrate_configured |= gps.setBaudrate(57600);
    }

    if (!baudrate_configured) {
      RTT::log( RTT::Error ) << "Could not configure baud rate for GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }

    // configure GPS receiver
    if (!gps.configure()) {
      RTT::log( RTT::Error ) << "Could not configure GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }

    // enable SBAS
    if (!gps.enableSBAS(true)) {
      RTT::log( RTT::Warning ) << "Could not enable SBAS mode for GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }

    // subscribe to messages
    if (gps.subscribe<ublox_msgs::NavPOSLLH>(boost::bind(&GPS::handleNavPOSLLH, this, _1), 1) == Callbacks::iterator()) {
      RTT::log( RTT::Warning ) << "Could not subscribe NavPOSLLH message from GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }
    if (gps.subscribe<ublox_msgs::NavVELNED>(boost::bind(&GPS::handleNavVELNED, this, _1), 1) == Callbacks::iterator()) {
      RTT::log( RTT::Warning ) << "Could not subscribe NavVELNED message from GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }
    if (gps.subscribe<ublox_msgs::NavSTATUS>(boost::bind(&GPS::handleNavSTATUS, this, _1), 4) == Callbacks::iterator()) {
      RTT::log( RTT::Warning ) << "Could not subscribe NavSTATUS message from GPS at port " << device << RTT::endlog();
      gps.close();
      continue;
    }

    // configuration successfull
    success = true;
    break;
  } while (--retries > 0);

  return success;
}

void GPS::handleNavPOSLLH(const ublox_msgs::NavPOSLLH& data) {
  fix.header.stamp = port->getTimestamp();
  fix.header.frame_id = frame_id;
  fix.latitude = data.lat * 1e-7;
  fix.longitude = data.lon * 1e-7;
  fix.altitude = data.height * 1e-3;
  if (status.gpsFix >= status.GPS_3D_FIX)
      fix.status.status = fix.status.STATUS_FIX;
  else
      fix.status.status = fix.status.STATUS_NO_FIX;

  fix.status.service = fix.status.SERVICE_GPS;
  portFix.write(fix);
}

void GPS::handleNavVELNED(const ublox_msgs::NavVELNED& data) {
  velocity.header.stamp = port->getTimestamp();
  velocity.header.frame_id = frame_id;
  velocity.vector.x =  data.velN * 1e-2;
  velocity.vector.y = -data.velE * 1e-2;
  velocity.vector.z = -data.velD * 1e-2;
  portVelocity.write(velocity);
}

void GPS::handleNavSTATUS(const ublox_msgs::NavSTATUS& data) {
  status = data;
}


void GPS::updateHook()
{
  RTT::Logger::In in(getName());
}

void GPS::cleanupHook() {
  RTT::Logger::In in(getName());
  gps.close();
  if (port) port->close();
}

bool GPS::configureDynModel(unsigned int dynModel) {
  ublox_msgs::CfgNAV5 cfg;
  cfg.dynModel = dynModel;
  cfg.mask = ublox_msgs::CfgNAV5::MASK_DYN;
  return gps.configure(cfg);
}

bool GPS::configureFixMode(unsigned int fixMode) {
  ublox_msgs::CfgNAV5 cfg;
  cfg.fixMode = fixMode;
  cfg.mask = ublox_msgs::CfgNAV5::MASK_FIX_MODE;
  return gps.configure(cfg);
}

GPS::Worker::Worker(System::BaseSerialPort *port)
  : port_(port)
  , buffer_length_(0)
{
  activity_.run(this);
  activity_.start();
}

GPS::Worker::~Worker()
{
  activity_.stop();
}

bool GPS::Worker::send(const unsigned char *data, const unsigned int size) {
  if (!port_) return false;
  int result = port_->send(data, size);
  return result > 0 && (unsigned int)result == size;
}

void GPS::Worker::wait(const boost::posix_time::time_duration& timeout) {
  if (!port_) return;
  port_->wait(timeout.total_milliseconds());
}

bool GPS::Worker::initialize() {
  if (!port_) return false;
  break_worker_ = false;
  return true;
}

void GPS::Worker::step()
{
  if (port_->wait(1000)) {
    int received = port_->receive(buffer_.data() + buffer_length_, buffer_.size() - buffer_length_);
    if (received > 0) {
      buffer_length_ += received;
      this->callback_(buffer_.data(), buffer_length_);
    }
  }
}

void GPS::Worker::loop()
{
  while(!break_worker_) step();
}

bool GPS::Worker::breakLoop()
{
  break_worker_ = true;
  return true;
}

void GPS::Worker::finalize()
{}

} // namespace ublox_gps
} // namespace uxvcos

ORO_CREATE_COMPONENT(uxvcos::ublox_gps::GPS)
