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
