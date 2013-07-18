#ifndef INTERFACE_ETHERNETSERIALPORT_H
#define INTERFACE_ETHERNETSERIALPORT_H

#include <uxvcos/Module.h>
#include <uxvcos/system/BaseSerialPort.h>

#include <uxvcos/stream/Buffer.h>

#include <rtt/Operation.hpp>
#include <rtt/Logger.hpp>

namespace uxvcos {
namespace Interface {

class EthernetSerialPort : public Module, public System::BaseSerialPort
{
public:
  class Handler {
  public:
    virtual ~Handler() {}
    virtual bool setBaudrate(EthernetSerialPort* port, unsigned long baudrate) = 0;
    virtual int uartSend(EthernetSerialPort* port, const void* source, unsigned int length) = 0;
    virtual bool uartPoll(EthernetSerialPort* port) = 0;
  };

  EthernetSerialPort(RTT::TaskContext* parent, int index, const std::string& name = "SerialPort", const std::string& description = "Interface to the Serial Port on the Ethernet Board");
  virtual ~EthernetSerialPort();

  virtual bool open();
  virtual void close();
  virtual bool isOpen() const;

  virtual bool blocking(bool b);
  virtual int send(const void *source, size_t size);

  virtual bool setDevice(std::string device);
  virtual bool setBaudrate(unsigned long b);

  virtual int receive(void *source, size_t size);
  virtual uxvcos::Time getTimestamp() const;
  virtual int bytesAvailable() const;

  virtual bool wait(unsigned long msecs);

  virtual bool initialize();
  virtual void cleanup();

  virtual bool receiveHandler(void* source, unsigned int length, const uxvcos::Time timestamp);

  int getIndex() const { return index; }

protected:
  Buffer<unsigned char> in;
  Handler *handler;
  int index;
  RTT::Property<unsigned int> baudrate;
  bool _isOpen;
  uxvcos::Time timestamp;

  // bool setBaudrate(unsigned int b);
  int sendString(std::string str);
  std::string receiveString();

private:
  class Worker;
  Worker *worker;

  bool poll();
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETSERIALPORT_H
