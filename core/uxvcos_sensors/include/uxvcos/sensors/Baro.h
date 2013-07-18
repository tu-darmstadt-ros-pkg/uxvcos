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

#ifndef UXVCOS_SENSOR_BARO_H
#define UXVCOS_SENSOR_BARO_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>
#include <uxvcos/filter/PTn.h>

#include <hector_std_msgs/typekit/UInt16.h>
#include <hector_uav_msgs/typekit/Altimeter.h>
#include <geometry_msgs/typekit/PointStamped.h>

#include <limits>

namespace uxvcos {
namespace Sensors {

class Baro : public RawSensor<hector_std_msgs::UInt16, hector_uav_msgs::Altimeter> {
public:
  Baro(RTT::TaskContext* parent, const std::string& name = "Baro", const std::string& port_name = "altimeter", const std::string& description = "Barometric Pressure")
    : RawSensor<hector_std_msgs::UInt16, hector_uav_msgs::Altimeter>(parent, name, port_name, description)
    , transformation("Baro")
    , filter("Filter", 0.0)
    , referencePressure("ReferencePressure", "The reference pressure to be used for altitude calculation", 1013.25)
  {
    properties()->addProperty(transformation);
    properties()->addProperty(filter);
    properties()->addProperty(referencePressure);

    this->addPort("pressure_height", portBaroHeight).doc("Output of sensor " + name);
    parent->addPort(portBaroHeight);
  }

  virtual bool convert(const hector_std_msgs::UInt16& raw) {
    if (raw.data == 0 || raw.data == 0xFFFF) return false;
    double temp = transformation.convert(raw.data);
    if (temp > 900.0 && temp < 1100.0) {
      data.pressure = filter.set(temp, raw.header.stamp.toSec());

      if ((data.pressure >= 100.0) && (data.pressure <= 1100.0))
        data.altitude = 44330.0 * (1.0 - pow(data.pressure / referencePressure.get(), 0.19));
      else
        data.altitude = std::numeric_limits<hector_uav_msgs::Altimeter::_altitude_type>::quiet_NaN();
      data.qnh = referencePressure.get();

      return true;
    } else {
      return false;
    }
  }

  virtual void updateHook() {
    baroHeight.header = data.header;
    baroHeight.point.z = data.altitude;
    portBaroHeight.write(baroHeight);
  }

protected:
  LinearTransformation<int,double> transformation;
  Filter::PTn<double> filter;

  RTT::Property<double> referencePressure;
  RTT::OutputPort<geometry_msgs::PointStamped> portBaroHeight;
  geometry_msgs::PointStamped baroHeight;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_BARO_H
