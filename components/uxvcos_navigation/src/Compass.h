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

#ifndef UXVCOS_NAVIGATION_COMPASS_H
#define UXVCOS_NAVIGATION_COMPASS_H

#include <uxvcos/Transformation.h>
#include <uxvcos/Module.h>

#include <geometry_msgs/typekit/Vector3Stamped.h>

#include "Magnetometer3D.h"

namespace uxvcos {
namespace Navigation {

class Compass : public Module {
public:
  Compass(RTT::TaskContext* parent, const std::string& name = "Compass", const std::string& description = "Compass Sensor")
    : Module(parent, name, description)
    , Bx("DeviationX")
    , By("DeviationY")
    , Bz("DeviationZ")
  {
     properties()->addProperty(Bx);
     properties()->addProperty(By);
     properties()->addProperty(Bz);
  }

  void SetMeasurement(const geometry_msgs::Vector3Stamped& magnetic, double roll, double pitch) {
    magnetic_field.header = magnetic.header;
    magnetic_field.vector.x = Bx.convert(magnetic.vector.x);
    magnetic_field.vector.y = By.convert(magnetic.vector.y);
    magnetic_field.vector.z = Bz.convert(magnetic.vector.z);

    magnetometer_.SetMeasurements(&magnetic_field.vector.x, &magnetic_field.vector.y, &magnetic_field.vector.z, &roll, &pitch);
  }

  TMAGNETOMETER3D& magnetometer() {
    return magnetometer_;
  }

  double GetMagHeading() { return magnetometer_.GetMagHeading(); }
  double GetGeoHeading() { return magnetometer_.GetGeoHeading(); }
  double GetDeclination() { return magnetometer_.GetDeclination(); }

  void GetNormalizedMagneticField(double &x, double &y, double &z)
  {
    double norm = sqrt(magnetic_field.vector.x*magnetic_field.vector.x + magnetic_field.vector.y*magnetic_field.vector.y + magnetic_field.vector.z*magnetic_field.vector.z);
    x = magnetic_field.vector.x / norm;
    y = magnetic_field.vector.y / norm;
    z = magnetic_field.vector.z / norm;
  }

protected:
  LinearTransformation<double,double> Bx, By, Bz;
  TMAGNETOMETER3D magnetometer_;

  geometry_msgs::Vector3Stamped magnetic_field;
};

} // namespace Navigation
} // namespace uxvcos

#endif // UXVCOS_NAVIGATION_COMPASS_H
