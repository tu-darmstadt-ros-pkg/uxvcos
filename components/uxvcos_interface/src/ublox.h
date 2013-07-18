//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

/*! @file ublox.h
 *  @brief uBlox decoding and encoding
 *
 *  functions used for decoding and encoding uBlox packets with respect to synchronization, checksums etc.
 */

#ifndef _UBLOX_H
#define _UBLOX_H

// individual structure alignment (linux- gcc)
#ifdef __GNUC__
 #ifndef GCC3X_PACK8
  #define GCC3X_PACK8 __attribute__((__packed__))
 #endif
#else
 #ifndef GCC3X_PACK8
  // individual structure alignment (windows visual studio 2005)
  #define GCC3X_PACK8
 #endif
 #pragma pack(push,1)
#endif

#ifdef __GNUC__
 #include <inttypes.h>
#else
typedef unsigned int size_t;
typedef int ssize_t;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
#endif

// general constants
#define UBLOX_MAXLENGTH  1024                //!< maximum length of uBlox packet
#define UBLOX_MAXPAYLOAD (UBLOX_MAXLENGTH-8)
#define UBLOX_FRAMESIZE		8

#define UBLOX_DEFAULT_SYNC       { 0x37, 0x42 }          //!< snchronization characters at the start of a uBlox frame
//extern char UBLOX_SYNC[];

// error constants
#define UBLOX_ERROR_CHECKSUM   1             //!< wrong checksum
#define UBLOX_ERROR_BIGPACKET  2             //!< packet too big (see UBLOX_MAXPAYLOADLENGTH)
#define UBLOX_ERROR_NOUBLOX    3             //!< frame invalid

// uBlox type definitions
typedef struct UBloxAttribute {
  uint8_t type;                                   //!< Type of attribute
  uint8_t length;                                 //!< Length of attribute value
  void *value;                                          //!< Pointer to attribute value
} GCC3X_PACK8 UBloxAttribute;

typedef struct UBloxPacket {
  char sync1;                                  //!< Synchronization character
  char sync2;                                  //!< Synchronization character
  uint8_t  classId;                               //!< Class identifier of the packet
  uint8_t  messageId;                             //!< Message identifier of the packet
  uint16_t payloadLength;                         //!< Length of the payload (fixed + variable part)
  char  payload[UBLOX_MAXPAYLOAD + 2];         //!< Payload (2 additional bytes for the checksum)
  char checksumA;                              //!< Checksum A
  char checksumB;                              //!< Checksum B
} GCC3X_PACK8 UBloxPacket;

typedef struct UBloxHeader {
  uint8_t  classId;                               //!< Class identifier of the packet
  uint8_t  messageId;                             //!< Message identifier of the packet
  uint16_t payloadLength;                         //!< Length of the payload (fixed + variable part)
} GCC3X_PACK8 UBloxHeader;

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*ublox_callback_function)(char *, size_t);

#ifdef _QUEUE_H
int ublox_findInQueue(Queue *q, UBloxHeader *ublox);
/*!< find the first uBlox packet in a queue. The preceding bytes will be removed from the queue.
    @param q      pointer to Queue object
    @param ublox if not NULL, pointer to a UBloxHeader struct, where header information will be stored
    @retval 0    no (full) packet found in queue
    @retval >0   length of the packet, packet is at top position in the queue now
 */

int ublox_decodeFromQueue(Queue *q, void *payload, size_t *payloadLength, UBloxHeader *ublox);
/*!< decode the first packet in a queue. The packet will be removed from the queue.
    @param q      pointer to Queue object
    @param payload pointer to a buffer, where payload will be copied to
    @param size   size of the payload buffer
    @param ublox if not NULL, pointer to a UBloxHeader struct, where header information will be stored
    @retval 0    no (full) packet found in queue
    @retval <0   decoding error (see Error Constants)
    @retval >0   length of the packet's payload
 */
#endif

int ublox_findNext(char **pos, size_t *size);
int ublox_find(char **pos, size_t *size, UBloxHeader *ublox);
 /*!< find the first uBlox packet in a queue. The preceding bytes will be removed from the queue.
    @param buffer  pointer to incoming uBlox buffer
    @param size    size of incoming buffer
    @param ublox if not NULL, pointer to a UBloxHeader struct, where header information will be stored
    @retval 0    no (full) packet found
    @retval >0   length of the packet, packet is at top position in the queue now
 */

int ublox_decodePacket(char *buffer, size_t size, UBloxPacket **packet);
int ublox_decode(char *buffer, size_t size, void **payload, size_t *payloadLength, UBloxHeader *ublox);
/*!< decode the first packet in a queue. The packet will be removed from the queue.
    @param buffer  pointer to incoming uBlox buffer
    @param size    size of incoming buffer
    @param payload pointer to a pointer to a buffer, will point to the payload in the buffer
    @param ublox if not NULL, pointer to a UBloxHeader struct, where header information will be stored
    @retval 0    no (full) packet found in queue
    @retval <0   decoding error (see Error Constants)
    @retval >0   length of the packet's payload
 */

int ublox_delete(char *buffer, size_t *size, char *pos);

int ublox_decodeAttributes(UBloxPacket *packet, size_t offset, UBloxAttribute* attributes, uint8_t attributesMax);
/*!< fill an array of UBloxAttribute structures out of a payload
    @param packet pointer to a uBlox packet structure
    @param offset size of the fixed part of the payload
    @param attributes array of UBloxAttribute structures
    @param attributesMax size of the array
    @retval <0   decoding error (see Error Constants)
    @retval >=0  number of attributes
 */

int ublox_encodeAttributes(UBloxPacket *packet, size_t offset, UBloxAttribute* attributes, uint8_t attributesCount);
/*!< serialize an array of UBloxAttribute structures
    @param packet pointer to a uBlox packet structure
    @param offset size of the fixed part of the payload
    @param attributes array of UBloxAttribute structures
    @param attributesCount number of attributes to be encoded
    @retval <0  encoding error (see Error Constants)
    @retval >=0  length of the attribute part of the payload (<= lengthMax)
 */

int ublox_sendPacket(UBloxPacket *packet, ublox_callback_function func);
/*!< compose and send a uBlox packet
    @param packet pointer to the uBlox packet to be send
    @param func   function used for sending
    @retval <0   an error occured (see Error Constants)
    @retval >=0  return value of *func
 */

int ublox_send(const void *message, size_t length, uint8_t classId, uint8_t messageId, ublox_callback_function func);
/*!< compose and send a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
    @param message pointer to the uBlox message to be send (the payload)
    @param length length of the message (= length of the payload)
    @param classId Class ID of the message
    @param messageId Message ID of the message
    @param func   function used for sending
    @retval <0   an error occured (see Error Constants)
    @retval >=0  return value of *func
 */

#ifdef _QUEUE_H
int ublox_sendToQueue(const void *message, size_t length, uint8_t classId, uint8_t messageId, Queue *q);
/*!< compose and send a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
    @param message pointer to the uBlox message to be send (the payload)
    @param length length of the message (= length of the payload)
    @param classId Class ID of the message
    @param messageId Message ID of the message
    @param q      pointer to Queue object
    @retval <0   an error occured (see Error Constants)
    @retval >=0  return value of qwrite
 */
#endif

int ublox_encode(const void *message, size_t length, uint8_t classId, uint8_t messageId, char *buffer, size_t size);
/*!< compose a uBlox packet using a predefined handler for the underlying layer (see UBLOX_SEND_FUNCTION)
    @param message pointer to the uBlox message to be send (the payload)
    @param length length of the message (= length of the payload)
    @param classId Class ID of the message
    @param messageId Message ID of the message
    @param buffer pointer to the buffer for the encoded packet (can be equal message pointer)
    @param size   size of the output buffer in bytes
    @retval <0   an error occured (see Error Constants)
    @retval >=0  return value of *func
    @see ublox_send
 */

const char *ublox_strerror(int err);
const char *ublox_strlasterror(void);

#ifdef __cplusplus
}
#endif

#ifndef __GNUC__
  #pragma pack(pop)
#endif

#endif // _UBLOX_H
