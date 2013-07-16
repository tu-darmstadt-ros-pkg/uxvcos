#ifndef STREAM_UBX_PROTOCOL_H
#define STREAM_UBX_PROTOCOL_H

#include <uxvcos.h>
#include <stdint.h>
#include <stdio.h>
#include <string>

#ifdef __GNUC__
	#define PACKED __attribute__((__packed__))
#else
	#define PACKED
	#pragma pack(push, 1)
#endif

namespace UBX {

static const unsigned char UBLOX_DEFAULT_SYNC[] = { 0x37, 0x42 }; //!< synchronization characters at the start of a uBlox frame
static const unsigned int UBLOX_MAXLENGTH = 4096;                 //!< maximum length of uBlox packet
static const unsigned int UBLOX_FRAMESIZE = 8;
static const unsigned int UBLOX_MAXPAYLOAD = (UBLOX_MAXLENGTH - UBLOX_FRAMESIZE);

enum Error { // error constants
  UBLOX_NO_ERROR = 0,                   //!< no error at all
  UBLOX_ERROR_CHECKSUM = 1,             //!< wrong checksum
  UBLOX_ERROR_BIGPACKET = 2,            //!< packet too big (see UBLOX_MAXPAYLOADLENGTH)
  UBLOX_ERROR_NOUBLOX = 3,              //!< frame invalid
};

struct PACKED Attribute {
  unsigned char type;                                   //!< Type of attribute
  unsigned char length;                                 //!< Length of attribute value
  void *value;                                    //!< Pointer to attribute value
};

struct PACKED Packet {
  unsigned char sync1;                                  //!< Synchronization character
  unsigned char sync2;                                  //!< Synchronization character
  unsigned char  classId;                               //!< Class identifier of the packet
  unsigned char  messageId;                             //!< Message identifier of the packet
  unsigned short payloadLength;                         //!< Length of the payload (fixed + variable part)
  unsigned char  payload[UBLOX_MAXPAYLOAD + 2];         //!< Payload (2 additional bytes for the checksum)
  unsigned char checksumA;                              //!< Checksum A
  unsigned char checksumB;                              //!< Checksum B
};

struct PACKED Header {
  unsigned char  classId;                               //!< Class identifier of the packet
  unsigned char  messageId;                             //!< Message identifier of the packet
  unsigned short payloadLength;                         //!< Length of the payload (fixed + variable part)
};

struct PACKED Checksum {
  unsigned char checksumA;                              //!< Checksum A
  unsigned char checksumB;                              //!< Checksum B

  Checksum() {
    reset();
  }

  void reset() {
    checksumA = 0;
    checksumB = 0;
  }

  bool operator==(const Checksum& other) {
    return (checksumA == other.checksumA) && (checksumB == other.checksumB);
  }

  void add(const unsigned char *data, unsigned length) {
    for(unsigned i = 0; i < length; i++) {
      checksumA = (unsigned char) (checksumA + *data++); checksumB = (unsigned char)(checksumA + checksumB);
    }
  }
};

class UXVCOS_API Protocol {
public:
  typedef int (*callback_function)(unsigned char *, unsigned);

  Protocol()
    : lastError(UBLOX_NO_ERROR)
  {
    UBLOX_SYNC[0] = UBLOX_DEFAULT_SYNC[0];
    UBLOX_SYNC[1] = UBLOX_DEFAULT_SYNC[1];
  }

  int findNext(unsigned char **pos, unsigned *size);
  int find(unsigned char **pos, unsigned *size, Header *header);

  int decodePacket(unsigned char *buffer, unsigned size, Packet **packet);
  int decode(unsigned char *buffer, unsigned size, void **payload, unsigned *payloadLength, Header *header);
  int remove(unsigned char *buffer, unsigned *size, unsigned char *pos);

  int decodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, unsigned char attributesMax);
  int encodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, unsigned char attributesCount);

  int sendPacket(Packet *packet, callback_function func);
  int send(const void *message, unsigned length, unsigned char classId, unsigned char messageId, callback_function func);
  int encode(const void *message, const unsigned length, const unsigned char classId, const unsigned char messageId, unsigned char *buffer, const unsigned size);

  bool setSync(const unsigned char* sync) { UBLOX_SYNC[0] = sync[0]; UBLOX_SYNC[1] = sync[1]; return true; }
  bool setSync(const std::string& sync);
  const unsigned char *getSync() { return UBLOX_SYNC; }

  UBX::Error getLastError() const { return lastError; }
  const char *strLastError() const { return strError(lastError); }

  static const char *strError(int err) {
    static const char *errstr[] = { "",
      "wrong checksum", // [UBLOX_ERROR_CHECKSUM]
      "illegal packet size", // [UBLOX_ERROR_BIGPACKET]
      "invalid uBlox frame", // [UBLOX_ERROR_NOUBLOX]
    };
    if (err < 0 || err > (int)(sizeof(errstr) / sizeof(*errstr))) return "";
    return errstr[-err];
  }

protected:
  unsigned char UBLOX_SYNC[2];          //!< synchronization characters at the start of a uBlox frame
  UBX::Error lastError;

  void calculate_checksum(Packet *packet);
  Checksum checksum(const Header *header, const unsigned char *message, unsigned length);

  int setLastError(UBX::Error c)
  {
    lastError = c;
    return -c;
  }
};

} // namespace UBX

#ifdef __GNUC__
#else
	#pragma pack(pop)
#endif

#endif // STREAM_UBX_PROTOCOL_H
