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

#ifndef UXVCOS_SENSOR_GPS_H
#define UXVCOS_SENSOR_GPS_H

#include <system/BaseSerialPort.h>
#include <stream/Buffer.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <sensor_msgs/typekit/NavSatFix.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

#include <string>

class TUBLOXGPS;

namespace uxvcos {
namespace Sensors {

class GPS : public RTT::TaskContext {
public:
  GPS(const std::string& name = "GPS", const std::string& description = "GPS Receiver");

  void setSerialPort(System::BaseSerialPort *port);
  std::string getDevice() const;

  bool configureDynModel(unsigned int dynModel);
  bool configureFixMode(unsigned int fixMode);

protected:
  virtual bool configureHook();
  
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();
  
  virtual void cleanupHook();

  // RTT::InputPort<Data::Navigation::RawGPS> rawIn;
//  RTT::OutputPort<Data::Navigation::RawGPS> rawOut;
//  RTT::OutputPort<Data::Navigation::GPS> gps;
//  RTT::OutputPort<Data::Navigation::SVInfo> svinfo;
//  RTT::OutputPort<Data::Navigation::SBASDGPS> sbasdgps;
  RTT::OutputPort<sensor_msgs::NavSatFix> portFix;
  RTT::OutputPort<geometry_msgs::Vector3Stamped> portVelocity;

  sensor_msgs::NavSatFix fix;
  geometry_msgs::Vector3Stamped velocity;

private:
  System::BaseSerialPort *port;
  TUBLOXGPS *UbloxGPS;
  std::string device;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_GPS_H
