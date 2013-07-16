#ifndef INTERFACE_VISTANAV_H
#define INTERFACE_VISTANAV_H

#include <interface/SerialInterface.h>

#include <sensors/IMU.h>
#include <sensors/Baro.h>
#include <types/navigation.h>

namespace Interface {
class VistaNav : public SerialInterface {
public:
  VistaNav(const std::string& name = "VistaNav");
  virtual ~VistaNav();

protected:
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

  virtual bool newData(const Data::Streamable* data);

protected:
  RTT::OutputPort<Data::Navigation::RawGPS> rawGPS;

  Sensor::IMU imu;
  Sensor::Baro baro;
};
} // namespace Interface

#endif // INTERFACE_VISTANAV_H
