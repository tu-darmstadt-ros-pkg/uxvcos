#include "system/BaseSerialPort.h"
#include "system/SerialPort.h"
#include "stream/Buffer.h"
#include "system/NameServer.h"

namespace {
  System::NameServer<System::BaseSerialPort *> nameServer;
}

namespace System {
  BaseSerialPort *getSerialPort(const std::string& device) {
    if (device.empty()) return 0;
    BaseSerialPort *serialPort = 0;

    serialPort = nameServer.getObject(device);

    if (!serialPort) serialPort = new SerialPort(device);
    return serialPort;
  }

  void registerSerialPort(BaseSerialPort* port, const std::string& device) {
    nameServer.registerObject(port, device);
  }

  void unregisterSerialPort(BaseSerialPort* port) {
    nameServer.unregisterObject(port);
  }

  void BaseSerialPort::flush() {
    if (writeBuffer) {
      send(writeBuffer->data(), writeBuffer->size());
      writeBuffer->clear();
    }
  }

  BaseStream::element_t *BaseSerialPort::data() {
    if (!readBuffer) readBuffer.reset(new Buffer<uint8_t>());
    return readBuffer->data();
  }

  BaseStream::element_t *BaseSerialPort::del(BaseStream::ssize_t len) {
    if (len < 0) return 0;
    if (!readBuffer) readBuffer.reset(new Buffer<uint8_t>());

    if (readBuffer->size() < (size_t) len) {
      size_t bytes_to_read = len - readBuffer->size();
      element_t *destination = readBuffer->add(bytes_to_read);
      if (!destination) return 0;
      if (receive(destination, bytes_to_read) != static_cast<int>(bytes_to_read)) return 0;
    }
    return readBuffer->del(len);
  }

  BaseStream::element_t *BaseSerialPort::tail() {
    if (!writeBuffer) writeBuffer.reset(new Buffer<uint8_t>());
    return writeBuffer->tail();
  }

  BaseStream::element_t *BaseSerialPort::add(BaseStream::ssize_t len) {
    if (len < 0) return 0;
    if (!writeBuffer) writeBuffer.reset(new Buffer<uint8_t>());
    return writeBuffer->add(len);
  }
}