#ifndef INTERFACE_ETHERNETAHRS_H
#define INTERFACE_ETHERNETAHRS_H

#include <uxvcos/sensors/Sensor.h>
#include "EthernetSensor.h"
#include <sensor_msgs/typekit/Imu.h>

namespace uxvcos {
namespace Interface {

typedef Sensors::Sensor<sensor_msgs::Imu> AHRS;

class EthernetAHRS : public EthernetSensor<AHRS>
{
public:
    EthernetAHRS(EthernetInterface* interface, const std::string& name = "AHRS", const std::string& port_name = "ahrs_imu", const std::string& description = "Attitude and Heading Reference System");
    virtual ~EthernetAHRS();

    virtual void Set(const ArmAttitude_t& attitude);
    virtual void Set(const ArmBias_t& bias);
    virtual void Set(const ArmVelocity_t& velocity);

    virtual void addTo(EthernetSensorContainer &sensors);
    virtual void request(Request &request) const;

    virtual bool decode(void *payload, size_t length, MessageId id);
    virtual std::ostream& operator>>(std::ostream& os);
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETAHRS_H
