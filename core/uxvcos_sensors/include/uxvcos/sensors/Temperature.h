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

#ifndef UXVCOS_SENSOR_TEMPERATURE_H
#define UXVCOS_SENSOR_TEMPERATURE_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>

#include <hector_std_msgs/typekit/UInt16.h>
#include <hector_std_msgs/typekit/Float64.h>

namespace uxvcos {
namespace Sensors {

class Temperature : public RawSensor<hector_std_msgs::UInt16, hector_std_msgs::Float64> {
public:
  Temperature(RTT::TaskContext* parent, const std::string& name = "Temperature", const std::string& port_name = "temperature", const std::string& description = "Temperature")
    : RawSensor<hector_std_msgs::UInt16, hector_std_msgs::Float64>(parent, name, port_name, description)
    , transformation("Temperature")
  {
    properties()->addProperty(transformation);
  }

  virtual bool convert(const hector_std_msgs::UInt16& raw) {
    data.data = transformation.convert(raw.data);
    return true;
  }

protected:
  LinearTransformation<int,double> transformation;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_TEMPERATURE_H
