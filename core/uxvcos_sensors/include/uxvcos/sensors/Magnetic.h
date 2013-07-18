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

#ifndef UXVCOS_SENSOR_MAGNETIC_H
#define UXVCOS_SENSOR_MAGNETIC_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>
#include <uxvcos/Mapping.h>

#include <hector_uav_msgs/typekit/RawMagnetic.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

namespace uxvcos {
namespace Sensors {

class Magnetic : public RawSensor<hector_uav_msgs::RawMagnetic, geometry_msgs::Vector3Stamped> {
public:
  Magnetic(RTT::TaskContext* parent, const std::string& name = "Magnetic", const std::string& port_name = "magnetic", const std::string& description = "Magnetic Field Sensor")
    : RawSensor<hector_uav_msgs::RawMagnetic, geometry_msgs::Vector3Stamped>(parent, name, port_name, description)
    , B0("B0")
    , B1("B1")
    , B2("B2")
    , DeviationX("DeviationX")
    , DeviationY("DeviationY")
    , DeviationZ("DeviationZ")
    , mapping(3)
  {
     data.header.frame_id = "/base_link";
     properties()->addProperty(B0);
     properties()->addProperty(B1);
     properties()->addProperty(B2);
     properties()->addProperty(DeviationX);
     properties()->addProperty(DeviationY);
     properties()->addProperty(DeviationZ);
     properties()->addProperty("Mapping", mapping).doc("Mapping of the channels to the x,y,z axes");
     properties()->addProperty("frame_id", data.header.frame_id);
  }

  virtual bool convert(const hector_uav_msgs::RawMagnetic& raw) {
    data.vector.x = B0.convert(raw.channel[0]);
    data.vector.y = B1.convert(raw.channel[1]);
    data.vector.z = B2.convert(raw.channel[2]);
    mapping(&data.vector.x);

    data.vector.x = DeviationX.convert(data.vector.x);
    data.vector.y = DeviationY.convert(data.vector.y);
    data.vector.z = DeviationZ.convert(data.vector.z);
    return true;
  }

protected:
  LinearTransformation<double,double> B0, B1, B2;
  LinearTransformation<double,double> DeviationX, DeviationY, DeviationZ;
  Mapping mapping;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_MAGNETIC_H
