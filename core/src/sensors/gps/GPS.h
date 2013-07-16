#ifndef UXVCOS_SENSOR_GPS_H
#define UXVCOS_SENSOR_GPS_H

#include <system/BaseSerialPort.h>
#include <stream/Buffer.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <sensor_msgs/typekit/NavSatFix.h>
#include <geometry_msgs/typekit/Vector3Stamped.h>

#include <string>

class TUBLOXGPS;

namespace uxvcos {
namespace Sensors {

class GPS : public RTT::TaskContext {
public:
  GPS(const std::string& name = "GPS", const std::string& description = "GPS Receiver");

  void setSerialPort(System::BaseSerialPort *port);
  std::string getDevice() const;

  bool configureDynModel(unsigned int dynModel);
  bool configureFixMode(unsigned int fixMode);

protected:
  virtual bool configureHook();
  
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();
  
  virtual void cleanupHook();

  // RTT::InputPort<Data::Navigation::RawGPS> rawIn;
//  RTT::OutputPort<Data::Navigation::RawGPS> rawOut;
//  RTT::OutputPort<Data::Navigation::GPS> gps;
//  RTT::OutputPort<Data::Navigation::SVInfo> svinfo;
//  RTT::OutputPort<Data::Navigation::SBASDGPS> sbasdgps;
  RTT::OutputPort<sensor_msgs::NavSatFix> portFix;
  RTT::OutputPort<geometry_msgs::Vector3Stamped> portVelocity;

  sensor_msgs::NavSatFix fix;
  geometry_msgs::Vector3Stamped velocity;

private:
  System::BaseSerialPort *port;
  TUBLOXGPS *UbloxGPS;
  std::string device;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_GPS_H
