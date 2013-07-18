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

#ifndef UXVCOS_SENSOR_ULTRASOUND_H
#define UXVCOS_SENSOR_ULTRASOUND_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/filter/PTn.h>

#include <hector_std_msgs/typekit/UInt16Array.h>
#include <sensor_msgs/typekit/Range.h>

namespace uxvcos {
namespace Sensors {

class Ultrasound : public RawSensor<hector_std_msgs::UInt16Array, sensor_msgs::Range> {
public:
  Ultrasound(RTT::TaskContext* parent, const std::string& name = "Ultrasound", const std::string& port_name = "sonar_height", const std::string& description = "Ultrasound Range Measurement Sensor")
    : RawSensor<hector_std_msgs::UInt16Array, sensor_msgs::Range>(parent, name, port_name, description)
    , filter("Filter", 0.0)
    , index(0)
    , field_of_view("field_of_view", "the size of the arc that the distance reading is valid for [rad]", 0.0)
    , min_range("min_range", "minimum range value [m]", 0.0)
    , max_range("max_range", "maximum range value [m]", 3.0)
  {
    properties()->addProperty(filter);
    properties()->addProperty(field_of_view);
    properties()->addProperty(min_range);
    properties()->addProperty(max_range);
  }

  virtual bool convert(const hector_std_msgs::UInt16Array& raw) {
    if (raw.data.size() <= index) return false;

    data.radiation_type = data.ULTRASOUND;
    data.field_of_view = field_of_view.get();
    data.min_range = min_range.get();
    data.max_range = max_range.get();

    if (raw.data[index] != 0 && raw.data[index] != 0xFFFF)
      data.range = filter.set(raw.data[index] * 0.000165, raw.header.stamp.toSec());
    else
      data.range = 0.0;

    return true;
  }

  std::size_t getIndex() const { return index; }
  void setIndex(std::size_t index) { this->index = index; }

protected:
  Filter::PTn<double> filter;
  std::size_t index;

  RTT::Property<double> field_of_view;
  RTT::Property<double> min_range;
  RTT::Property<double> max_range;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_ULTRASOUND_H
