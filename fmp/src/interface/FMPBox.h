#ifndef INTERFACE_FMPBOX_H
#define INTERFACE_FMPBOX_H

#include <interface/SerialInterface.h>

#include <sensors/Baro.h>
#include <sensors/FMP.h>

namespace uxvcos {
namespace Interface {

class FMPBox : public SerialInterface {
public:
  FMPBox(const std::string& name = "FMPBox");
  virtual ~FMPBox();

protected:
  virtual bool configureHook();
  virtual void cleanupHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

  void callback(const fmp_msgs::RawData& raw, const StreamConnectionBase *source);
  // virtual bool newData(const Data::Streamable* data);

protected:
  Sensors::FMP sensor;
  RTT::OutputPort<hector_uav_msgs::Altimeter> baro;
  Time last_update_;

  RTT::Property<double> qnh;
  RTT::Property<Duration> timeout;
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_FMPBOX_H
