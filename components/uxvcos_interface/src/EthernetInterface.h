#ifndef INTERFACE_ETHERNETINTERFACE_H
#define INTERFACE_ETHERNETINTERFACE_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Operation.hpp>
#include <rtt/os/Mutex.hpp>

#include <base/ModuleContainer.h>

#include <stream/Stream.h>
#include <system/Socket.h>
#include <system/EthernetAddress.h>
#include <system/BaseSerialPort.h>

#include <interface/BaseInterface.h>
#include <interface/RateControl.h>
#include "Motor.h"
#include "Servos.h"
#include "EthernetSerialPort.h"
#include "EthernetSensor.h"

#include "arm-interface.h"

namespace uxvcos {
namespace Interface {

class EthernetInterface
  : public BaseInterface
  , public Motor::Handler
  , public Servos::Handler
  , public EthernetSerialPort::Handler
{
public:
  typedef enum { MODE_ETHERNET, MODE_SERIAL } ConnectionType;

  EthernetInterface(const std::string &name = "Interface");
  virtual ~EthernetInterface();

  static int setup();
  void setupSensors();

  template <typename SensorType> boost::shared_ptr<SensorType> addSensor(SensorType *s);

protected:
  virtual bool configureHook();
  virtual void cleanupHook();
  virtual bool startHook();
  virtual void stopHook();
  virtual void updateHook();

protected:
  RateControl rateControl;
  EthernetSensorContainer sensors;

  boost::shared_ptr<Motor> motor;
  boost::shared_ptr<Servos> servos;
  boost::shared_ptr<EthernetSerialPort> uart0, uart1;

  RTT::Property<int> connectionType;
  RTT::Property<std::string> device;
  RTT::Property<std::string> destination;
  RTT::Property<unsigned int> protocol;
  RTT::Property<unsigned int> baudrate;
  RTT::Property<int> recv_timeout;

  unsigned long timeoutCounter;

  RTT::Property<RTT::PropertyBag> debugProperties;
  RateControl rateDebug;

  RTT::Operation<bool(unsigned int)> requestMethod;

  RTT::Attribute<double> interfaceDelay;
  RTT::Attribute<double> overallDelay;

public:
  bool send(const void *data, size_t size, unsigned char classId, unsigned char messageId);
  RateControl *getRateControl();
  RateControl::counter_t& getCounter();
  bool debugEnabled() const;

protected:
  bool send(const char *buffer, int length);
  bool request(unsigned int request);
  bool sendPeriodicRequest();
  bool receiveAnswer(bool retry = false);

  virtual bool newMotorCommand(const hector_uav_msgs::MotorPWM& motorCommand);
  virtual bool newServoCommand(const hector_uav_msgs::ServoCommand& servoCommand);
  virtual bool setBaudrate(EthernetSerialPort* port, unsigned long baudrate);
  virtual int uartSend(EthernetSerialPort* port, const void* source, unsigned int length);
  virtual bool uartPoll(EthernetSerialPort* port);

  RTT::Operation<bool(void)> checkVersionMethod;
  bool checkVersion();
 
private:
  System::Socket *socket;
  System::BaseSerialPort *serialPort;
  Stream *stream;

  System::EthernetAddress localAddress;
  System::EthernetAddress remoteAddress;
  
  Request theRequest;
  RTT::os::TimeService::ticks sendTimestamp;

  bool debug;

  char buffer_out[1500];
  Buffer<> buffer_in;

  char debugBuffer[256];

  RTT::os::Mutex requestMutex;
  RTT::os::Mutex sendMutex;
};

template <typename SensorType> boost::shared_ptr<SensorType> EthernetInterface::addSensor(SensorType *instance)
{
  boost::shared_ptr<SensorType> sensor = addModule(instance);
  if (!sensor->sensor()) {
    RTT::log(RTT::Fatal) << sensor->getName() << " is not a valid child of EthernetSensorBase" << RTT::endlog();
    if (sensor->module()) {
      RTT::log(RTT::Fatal) << "But it is a module.... hmmm...?!?" << RTT::endlog();
    }
    return sensor;
  }
  sensor->addTo(sensors);
  return sensor;
}

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETINTERFACE_H
