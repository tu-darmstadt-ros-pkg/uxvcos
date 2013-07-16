#ifndef UXVCOS_NAVIGATION_COMPASS_H
#define UXVCOS_NAVIGATION_COMPASS_H

#include <base/Transformation.h>
#include <base/Module.h>

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
