#ifndef INTERFACE_ETHERNETIMU_H
#define INTERFACE_ETHERNETIMU_H

#include <sensors/IMU.h>
#include "EthernetSensor.h"

namespace uxvcos {
namespace Interface {

class EthernetIMU : public EthernetSensor<Sensors::IMU> {
public:
  EthernetIMU(EthernetInterface *interface, const std::string& name = "IMU", const std::string& port_name = "raw_imu", const std::string& description = "Inertial Measurement Unit");
  virtual ~EthernetIMU();

  virtual bool sendConfiguration();
  virtual bool setZeroCommandExecute(double seconds);

  virtual void addTo(EthernetSensorContainer &sensors);
  virtual void request(Request &request) const;

  virtual bool decode(void *payload, size_t length, MessageId id);
  virtual std::ostream& operator>>(std::ostream& os);

protected:
  virtual bool initialize();

private:
  bool modeRaw;
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETIMU_H
