#ifndef COMMUNICATION_XBEE_H
#define COMMUNICATION_XBEE_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <stream/Stream.h>
#include <stream/Buffer.h>

#include <system/BaseSerialPort.h>

#include "XBeeAPI.h"

namespace Communication {
class XBee : public RTT::TaskContext
{
public:
  XBee(const std::string &name = "XBee")
    : RTT::TaskContext(name, PreOperational)
    , inBuffer(2048)
    , outBuffer(2048)
    , xbeeMY("xbeeMY", "XBee Source Address (0 - 0xFFFF)", 0)
    , xbeeCH("xbeeCH", "XBee Channel (0x0C - 0x17)", 0x17)
    , xbeeID("xbeeID", "XBee PAN ID (0 - 0xFFFF)", 0x3332)
    , verbosity("verbosity", "Verbosity of XBee Module", 0)
    , port(0)
  {
    this->properties()->addProperty(xbeeMY);
    this->properties()->addProperty(xbeeCH);
    this->properties()->addProperty(xbeeID);

    this->properties()->addProperty(verbosity);
  }

  ~XBee() {
    stop();
    cleanup();
  }

protected:
  bool configureHook();
  void cleanupHook();
  bool startHook();
  void updateHook();
  void stopHook();

  void send();

  void setPort(System::BaseSerialPort *port) {
    this->port = port;
  }

private:
  unsigned char xbeeFrameIdCounter;
  unsigned char xbeeFrameIdConfirmed;
  unsigned char xbeeFrameIdFailed;

  XBeePacketSendStatus sendStatus;
  XBeePacketATResponse ATResponse;
  time_t sendTime;

  bool initdone;
  bool foundStartDelimiter;

  Buffer<unsigned char> inBuffer;
  Buffer<unsigned char> outBuffer;
  unsigned char payload[100];

  int xbee_verify_checksum(void *packet, unsigned short length, unsigned char checksum);
  unsigned char xbee_add_checksum(void *packet, unsigned short length);

  int xbee_decode(Buffer<unsigned char> *buffer, unsigned char *payload, unsigned short payloadSize, XBeePacketReceive *packetReceive);
  int xbee_encode(void *payload, unsigned short payloadLength, unsigned char *buffer, unsigned short bufferSize, unsigned short destinationAddress, XBeePacketSend **packetSend);
  int xbee_sendStatus();
  int xbee_init(unsigned short MY, unsigned char CH, unsigned short ID, unsigned char *buffer, unsigned short bufferSize);
  int xbee_at(const char *command, const void *value, const int size, unsigned char *buffer, unsigned short bufferSize, unsigned char *frameId);

protected:
  RTT::Property<unsigned int> xbeeMY;
  RTT::Property<unsigned int> xbeeCH;
  RTT::Property<unsigned int> xbeeID;

  RTT::Property<unsigned int> verbosity;

  System::BaseSerialPort *port;
};
} // namespace Communication

#endif // COMMUNICATION_XBEE_H
