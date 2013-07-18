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

#include "FMPBox.h"

#include <rtt/Logger.hpp>
#include <rtt/Activity.hpp>
#include <sensors/FMP.h>

#include <hector_uav_msgs/Altimeter/pressure_height.h>

#include <rtt/Component.hpp>

namespace uxvcos {
namespace Interface {

FMPBox::FMPBox(const std::string& name)
  : SerialInterface(name)
  , sensor(this)
  , baro("altimeter")
  , qnh("qnh")
  , timeout("timeout", "Timeout after which the component switches over to the error state", Duration(1.0))
{
  this->ports()->addPort( baro );
  this->addProperty(qnh);
  this->addProperty(timeout);

  this->addConnection<fmp_msgs::RawData>(Data::Key(0x7B, 0x6F), "fmp", &FMPBox::callback, this);

  this->setActivity(new RTT::Activity(RTT::os::HighestPriority, 0, this->getName()));
}

FMPBox::~FMPBox()
{
  stop();
  cleanup();
}

bool FMPBox::configureHook() {
  RTT::Logger::In in(getName());
  if (!SerialInterface::configureHook()) return false;
  return true;
}

void FMPBox::cleanupHook() {
  RTT::Logger::In in(getName());
  SerialInterface::cleanupHook();
}

bool FMPBox::startHook() {
  RTT::Logger::In in(getName());
  if (!SerialInterface::startHook()) return false;

  last_update_ = Time::now();

  // RTT::log( RTT::Info ) << "Successfully started FMPBox interface" << RTT::endlog();
  return true;
}

void FMPBox::stopHook() {
  RTT::Logger::In in(getName());
  SerialInterface::stopHook();
}

void FMPBox::updateHook() {
  RTT::Logger::In in(getName());
  SerialInterface::updateHook();

  if (!last_update_.isZero() && (Time::now() - last_update_) > timeout) {
    error();
  }
}

void FMPBox::callback(const fmp_msgs::RawData& raw, const StreamConnectionBase *source) {
  sensor.Set(raw, Time::now());

  hector_uav_msgs::Altimeter altimeter;
  altimeter.header.stamp = sensor.getTimestamp();
  altimeter.pressure = sensor.get().pStat;
  altimeter.qnh = qnh;
  if (!std::isnan(altimeter.pressure)) {
    hector_uav_msgs::altitudeFromPressure(altimeter);
    baro.write(altimeter);
  }

  last_update_ = altimeter.header.stamp;
}

ORO_CREATE_COMPONENT(FMPBox)

} // namespace Interface
} // namespace uxvcos
