#include <rtt/Logger.hpp>

#ifdef __GNUC__
	#include <netinet/in.h>
#else
	#include <winsock2.h>
#endif

#include <string>
#include <system/systemcalls.h>

#include "XBee.h"

using namespace RTT;

namespace Communication {

bool XBee::configureHook() {
  initdone = false;
  return true;
}

void XBee::cleanupHook() {
}

bool XBee::startHook() {
  if (!port) return false;
  return true;
}

void XBee::stopHook() {
  this->disconnect();
}

void XBee::updateHook() {
  unsigned int verbose = verbosity.get();

  if (!initdone) {
    int ret = xbee_init(xbeeMY.get(), xbeeCH.get(), xbeeID.get(), outBuffer.data(), outBuffer.free());
    if (ret > 0) {
      outBuffer.add(ret);
      send();
      this->getActivity()->trigger();
    } else if (ret < -1) {
      log( Error ) << "Could not communicate with XBee Module! Please check if it is connected properly and all configuration values are valid." << endlog();
      error();
    } else if (ret == -1) {
      initdone = true;
    }
  }

  XBeePacketReceive packetReceive;
  int readlen, writelen;

  inBuffer << (*port);

  while((readlen = xbee_decode(&inBuffer, payload, sizeof(payload), &packetReceive))) {
    if (readlen < 0) {
      if (verbose > 0) log( Error ) << "xbee_decode: error " << -readlen << endlog();
      continue;
    }

    if (verbose > 2) {
      char s[128];
      snprintf(s, sizeof(s)-1, "NIP <== (sourceAddress = 0x%04x, rssi = 0x%02x, options = 0x%02x) ", packetReceive.sourceAddress, packetReceive.rssi, packetReceive.options);
      log( Debug ) << s << endlog();
    }

    //writelen = nip2ip(readbuf, readlen, writebuf, sizeof(writebuf), &packetReceive);
    writelen = 0;

    if (writelen < 0) {
      continue;
    } else if (writelen == 0) {
      continue;
    }
  }
}

void XBee::send() {
  outBuffer >> (*port);
}

int XBee::xbee_verify_checksum(void *packet, unsigned short length, unsigned char checksum) {
  unsigned char* p;
  for (p = (unsigned char *) packet; p < (unsigned char *) packet + length; p++) checksum += *p;
  return (checksum != 0xFF);
}

unsigned char XBee::xbee_add_checksum(void *packet, unsigned short length) {
  unsigned char *p;
  unsigned char checksum = 0;
  for (p = (unsigned char *) packet; p < (unsigned char *) packet + length; p++) checksum += *p;
  return checksum;
}

int XBee::xbee_decode(Buffer<unsigned char> *buffer, unsigned char *payload, unsigned short payloadSize, XBeePacketReceive* packetReceive) {
  unsigned short length;
  unsigned char apiId;
  unsigned char checksum = 0;
  XBeePacketSendStatus sendStatus;

  // find Start Delimiter
  if (!foundStartDelimiter) {
    if (!buffer->find(0x7E)) return 0;
    foundStartDelimiter = true;
  }

  if (buffer->size() < 3) return 0; // read at least 4 bytes
  length = *((unsigned short *)(buffer->data()));
  ntohs(length);
  if (buffer->size() < (size_t) length + 3) return 0;

  // we have a full packet
  buffer->read(0,2); // read first 2 bytes (length) from queue

  // the remaining part depends on the apiId
  apiId = *(buffer->data());
  switch (apiId) {
    case 0x81: // RX Packet 16-bit Address
      buffer->read(packetReceive, sizeof(*packetReceive));
      checksum += xbee_add_checksum(packetReceive, sizeof(*packetReceive));

      length -= sizeof(*packetReceive); // substract 5 bytes API header
      if (length > payloadSize) return -XBEE_ERROR_BIGPACKET;
      buffer->read(payload, length);
      checksum += xbee_add_checksum(payload, length);

      checksum = 0xFF - checksum;
      if (buffer->get() != checksum) return -XBEE_ERROR_CHECKSUM;
      packetReceive->sourceAddress = ntohs(packetReceive->sourceAddress);
      // printf("zigBee RX packet from %04x (apiId = %02x, checksum = %02x, %d bytes)\n", packetReceive->sourceAddress, apiId, checksum, length + 4); fflush(NULL);
      return length;

    case 0x89: // TX Status
      if (length != sizeof(sendStatus)) return -XBEE_ERROR_BIGPACKET;
      buffer->read(&sendStatus, sizeof(sendStatus));
      checksum = buffer->get();

      if (xbee_verify_checksum(&sendStatus, length, checksum)) return -XBEE_ERROR_CHECKSUM;
      if (XBEE_DEBUG > 1 || (XBEE_DEBUG > 0 && sendStatus.status > 0))
        fprintf(stderr, "==> XBee Send Status: %d returned on frame ID %02x\n", sendStatus.status, sendStatus.frameId);
      if (sendStatus.status > 0)
        xbeeFrameIdFailed = sendStatus.frameId;
      else
        xbeeFrameIdConfirmed = sendStatus.frameId;
      return 0;

    case 0x88: // AT Command Response
      if (length > sizeof(ATResponse)) return -XBEE_ERROR_BIGPACKET;
      buffer->read(&ATResponse, length);
      checksum = buffer->get();

      if (xbee_verify_checksum(&ATResponse, length, checksum)) return -XBEE_ERROR_CHECKSUM;
      if (XBEE_DEBUG > 1 || (XBEE_DEBUG > 0 && ATResponse.status > 0))
        fprintf(stderr, "==> XBee AT Command Response: %d returned on frame ID %02x\n", ATResponse.status, ATResponse.frameId);
      return 0;
  }

  if (XBEE_DEBUG > 1) { fprintf(stderr, "xbee_decode: unknown packet"); }
  buffer->read(0, length + 1); // length + checksum
  // return -XBEE_ERROR_UNKNOWN;
  return 0;
}

int XBee::xbee_encode(void *payload, unsigned short payloadLength, unsigned char *buffer, unsigned short bufferSize, unsigned short destinationAddress, XBeePacketSend **packetSend) {
  XBeePacketSend *packet;

  if (payloadLength + sizeof(*packet) + 4 > bufferSize) return -XBEE_ERROR_BIGPACKET;

  *buffer++ = 0x7E;
  *buffer++ = (payloadLength + sizeof(*packet)) >> 8;
  *buffer++ = (payloadLength + sizeof(*packet)) & 0xFF;

  packet = (XBeePacketSend *) buffer;
  packet->apiId = 0x01;
  if (++xbeeFrameIdCounter == 0) xbeeFrameIdCounter = 1;
  packet->frameId = xbeeFrameIdCounter;
  *((unsigned short *) packet->destinationAddress) = htons(destinationAddress);
  packet->options = 0x00;
  // if (destinationAddress == 0xFFFF) packet->options = 0x04;
  buffer += sizeof(*packet);

  if (payload != NULL) memcpy(buffer, payload, payloadLength);
  buffer += payloadLength;
  *buffer++ = 0xFF - xbee_add_checksum(packet, payloadLength + sizeof(*packet));

  if (packetSend != NULL) *packetSend = packet;
  sendTime = time(NULL);
  return payloadLength + sizeof(*packet) + 4;
}

int XBee::xbee_sendStatus() {
  if (xbeeFrameIdCounter == sendStatus.frameId) return sendStatus.status;

  if (time(NULL) - sendTime <= XBEE_TIMEOUT)
    return -1;
  else
    return -2;
}

int XBee::xbee_init(unsigned short MY, unsigned char CH, unsigned short ID, unsigned char *buffer, unsigned short bufferSize) {
  static unsigned char step = 0;
  static unsigned char retry = 0;
  static unsigned char frameId;
  static time_t startTime = 0;
  const unsigned char zero = 0;

  if (startTime > 0 && ATResponse.frameId == frameId) {
    startTime = 0;
    if (ATResponse.status > 0) {
      return -2;
    } else {
      step++;
      retry = 0;
    }
  }

  if (startTime > 0 && time(NULL) - startTime > XBEE_TIMEOUT) {
    if (retry++ == XBEE_RETRIES) return -3;
    startTime = 0;
  }

  switch(step) {
    case 0:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("FR", NULL, 0, buffer, bufferSize, &frameId);
      }
      break;

    case 1:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("CE", &zero, sizeof(zero), buffer, bufferSize, &frameId);
      }
      break;

    case 2:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("A1", &zero, sizeof(zero), buffer, bufferSize, &frameId);
      }
      break;

    case 3:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("MY", &MY, sizeof(MY), buffer, bufferSize, &frameId);
      }
      break;

    case 4:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("CH", &CH, sizeof(CH), buffer, bufferSize, &frameId);
      }
      break;

    case 5:
      if (startTime == 0) {
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("ID", &ID, sizeof(ID), buffer, bufferSize, &frameId);
      }
      break;

    case 6:
      if (startTime == 0) {
        unsigned char level = 4;
        startTime = time(NULL);
        memset(&ATResponse, 0, sizeof(ATResponse));
        return xbee_at("PL", &level, sizeof(level), buffer, bufferSize, &frameId);
      }
      break;

    default:
      return -1;
  }

  return 0;
}

int XBee::xbee_at(const char *command, const void *value, const int size, unsigned char *buffer, unsigned short bufferSize, unsigned char *frameId) {
  unsigned char *check;
  int i;

  if (size + 8 > bufferSize) return -XBEE_ERROR_BIGPACKET;

  *buffer++ = 0x7E;
  *buffer++ = (size + 4) >> 8;
  *buffer++ = (size + 4) & 0xFF;
  check = buffer;

  *buffer++ = 0x08;
  if (++xbeeFrameIdCounter == 0) xbeeFrameIdCounter = 1;
  *buffer++ = xbeeFrameIdCounter;
  *buffer++ = *command;
  *buffer++ = *(command + 1);
  for (i = size - 1; i >= 0; i--) {
    *buffer++ = *(((char *) value) + i);
  }
  *buffer++ = 0xFF - xbee_add_checksum(check, size + 4);

  if (frameId != NULL) *frameId = xbeeFrameIdCounter;
  return size + 8;
}

} // namespace Communication

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( Communication::XBee )
