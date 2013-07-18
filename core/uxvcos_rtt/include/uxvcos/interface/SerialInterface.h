#ifndef INTERFACE_SERIALINTERFACE_H
#define INTERFACE_SERIALINTERFACE_H

#include <uxvcos/system/BaseSerialPort.h>
#include "StreamInterface.h"

namespace uxvcos {
namespace Interface {

class SerialInterface : public StreamInterface {
public:
  SerialInterface(const std::string& name = "SerialInterface");
  virtual ~SerialInterface();

protected:
  virtual bool configureHook();
  virtual bool startHook();
  // virtual void updateHook();
  virtual void stopHook();
  virtual void cleanupHook();

protected:
  RTT::Property<std::string> deviceName;
  RTT::Property<unsigned int> baudRate;

protected:
  System::BaseSerialPort *serialPort;
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_SERIALINTERFACE_H
