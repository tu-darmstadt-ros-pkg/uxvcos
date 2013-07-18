#ifndef INTERFACE_RC_H
#define INTERFACE_RC_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>
#include <uxvcos/Configuration.h>

#include <hector_uav_msgs/typekit/RawRC.h>
#include <hector_uav_msgs/typekit/RC.h>

#include <vector>

namespace uxvcos {
namespace Interface {

class RC : public Sensors::RawSensor<hector_uav_msgs::RawRC,hector_uav_msgs::RC> {
public:
  RC(RTT::TaskContext* parent, const std::string& name = "RC", const std::string& port_name = "rc", const std::string& description = "Radio Control Receiver");

  bool convert(const hector_uav_msgs::RawRC &update);

protected:
  bool initialize();
  void cleanup();

protected:
  Configuration configuration;
  class Axis;
  class Switch;
  std::vector<boost::shared_ptr<Axis> > axes;
  std::vector<boost::shared_ptr<Switch> > switches;
};

std::ostream& operator<<(std::ostream& os, const RC& rc);

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_RC_H
