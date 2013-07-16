#ifndef UXVCOS_SENSOR_BARO_H
#define UXVCOS_SENSOR_BARO_H

#include <sensors/Sensor.h>
#include <base/Transformation.h>
#include <filter/PTn.h>

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
