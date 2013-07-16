#ifndef UXVCOS_SENSOR_SUPPLY_H
#define UXVCOS_SENSOR_SUPPLY_H

#include <sensors/Sensor.h>
#include <hector_uav_msgs/typekit/Supply.h>

namespace uxvcos {
namespace Sensors {

class Supply : public Sensor<hector_uav_msgs::Supply> {
public:
  Supply(RTT::TaskContext* parent, const std::string& name = "Supply", const std::string& port_name = "supply", const std::string& description = "Supply")
    : Sensor<hector_uav_msgs::Supply>(parent, name, port_name, description)
  {}
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_SUPPLY_H
