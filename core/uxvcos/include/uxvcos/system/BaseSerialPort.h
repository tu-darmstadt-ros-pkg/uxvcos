#ifndef SYSTEM_BASESERIALPORT_H
#define SYSTEM_BASESERIALPORT_H

#include <string>
#include <stdexcept>

#include <uxvcos/stream/Stream.h>
#include <uxvcos/Time.h>

#include <uxvcos/uxvcos.h>

template <typename T> class Buffer;

namespace System {

class UXVCOS_API BaseSerialPort : public Stream {
public:
  BaseSerialPort() {}
  virtual ~BaseSerialPort() {}

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool blocking(bool) = 0;
  virtual bool isOpen() const = 0;

  virtual int send(const void *source, size_t size) = 0;
  virtual int receive(void *source, size_t size) = 0;
  virtual int bytesAvailable() const = 0;

  virtual bool wait(unsigned long msecs) = 0;
  virtual void flush();

  virtual bool setDevice(std::string device) = 0;
  virtual bool setBaudrate(unsigned long b) = 0;
  virtual uxvcos::Time getTimestamp() const { return uxvcos::Time(); }

  virtual bool setRequestToSend(bool enable) const     { return false; }
  virtual bool getClearToSend() const                  { return false; }
  virtual bool setDataTerminalReady(bool enable) const { return false; }
  virtual bool getDataSetReady() const                 { return false; }
  virtual bool getCarrierDetect() const                { return false; }
  virtual bool getRingIndicator() const                { return false; }

  // InStream members
  virtual element_t get() {
    element_t c;
    if (receive(&c, sizeof(c)) == sizeof(c)) return c;
    return EoF;
  }
  virtual size_t readsome(void *destination, size_t n) {
    int ret = receive(destination, n);
    _eof = (ret < 0);
    if (ret < 0) return 0;
    return ret;
  }

//  virtual size_t size() const {
//    return bytesAvailable();
//  }

  // OutStream members
  virtual OutStream& write(const void *source, size_t size) {
    int ret = send(source, size);
    _overflow = (ret < 0 || (size_t) ret != size);
    return *this;
  };

  // Stream members
  virtual bool good() const {
    return isOpen() && Stream::good();
  }

  virtual void reset() {
    BaseStream::reset();
    readBuffer.reset();
    writeBuffer.reset();
  }

  class UXVCOS_API Exception : public std::exception {
  public:
    Exception(const std::string& reason) throw() :
      reason(reason) {}
    virtual ~Exception() throw() {}
    virtual const char* what() const throw() { return reason.c_str(); }
  private:
    std::string reason;
  };

  // ROS compatibility methods
  virtual element_t *data();
  virtual element_t *del(ssize_t len);
  virtual element_t *tail();
  virtual element_t *add(ssize_t len);

private:
  boost::shared_ptr<Buffer<uint8_t> > readBuffer;
  boost::shared_ptr<Buffer<uint8_t> > writeBuffer;
};

UXVCOS_API BaseSerialPort *getSerialPort(const std::string& device);
UXVCOS_API void registerSerialPort(BaseSerialPort* port, const std::string& device);
UXVCOS_API void unregisterSerialPort(BaseSerialPort* port);

} // namespace System

#endif // SYSTEM_BASESERIALPORT_H
