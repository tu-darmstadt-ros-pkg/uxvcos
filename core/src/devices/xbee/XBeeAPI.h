#ifndef COMMUNICATION_XBEEAPI_H
#define COMMUNICATION_XBEEAPI_H

#undef PACKED
#ifdef __GNUC__
  #define PACKED __attribute__((__packed__))
#else
  #define PACKED
  #pragma pack(push, 1)
#endif

#define XBEE_ERROR_CHECKSUM   1
#define XBEE_ERROR_BIGPACKET  2
#define XBEE_ERROR_UNKNOWN    3
#define XBEE_ERROR_TXFAILED   4

#define XBEE_HEADERLENGTH_RECEIVE 8
#define XBEE_HEADERLENGTH_SEND 8
#define XBEE_PAYLOADLENGTH   100
#define XBEE_TIMEOUT          1 // seconds
#define XBEE_RETRIES          3

#define XBEE_STATUS_SUCCESS  0
#define XBEE_STATUS_NOACK   1
#define XBEE_STATUS_CCAFAILURE 2
#define XBEE_STATUS_PURGED  3

#define XBEE_DEBUG  1

typedef struct {
  unsigned char apiId;
  unsigned short sourceAddress;
  unsigned char rssi;
  unsigned char options;
} PACKED XBeePacketReceive;

typedef struct {
  unsigned char apiId;
  unsigned char frameId;
  unsigned char destinationAddress[2];
  unsigned char options;
} PACKED XBeePacketSend;

typedef struct {
  unsigned char apiId;
  unsigned char frameId;
  unsigned char status;
} PACKED XBeePacketSendStatus;

typedef struct {
  unsigned char apiId;
  unsigned char frameId;
  char command[2];
  unsigned char status;
  char value[8];
} PACKED XBeePacketATResponse;

#ifdef __GNUC__
#else
  #pragma pack(pop)
#endif

#endif // COMMUNICATION_XBEEAPI_H
