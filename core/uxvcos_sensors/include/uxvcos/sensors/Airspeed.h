#ifndef UXVCOS_SENSOR_AIRSPEED_H
#define UXVCOS_SENSOR_AIRSPEED_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>

#include <hector_std_msgs/typekit/UInt16.h>
#include <hector_std_msgs/typekit/Float64.h>

namespace uxvcos {
namespace Sensors {

class Airspeed : public RawSensor<hector_std_msgs::UInt16, hector_std_msgs::Float64> {
public:
  Airspeed(RTT::TaskContext* parent, const std::string& name = "Airspeed", const std::string& port_name = "airspeed", const std::string& description = "Airspeed")
    : RawSensor<hector_std_msgs::UInt16, hector_std_msgs::Float64>(parent, name, port_name, description)
    , transformation("Airspeed")
  {
    properties()->addProperty(transformation);
  }

  virtual bool convert(const hector_std_msgs::UInt16& raw) {
    data.data = transformation.convert(raw.data);
    return true;
  }

protected:
  QuadraticTransformation<int,double> transformation;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_AIRSPEED_H
