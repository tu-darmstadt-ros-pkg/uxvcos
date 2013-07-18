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

#include <string.h>
#include <stdlib.h>

#include "ublox.h"

// internal variables
static char UBLOX_SYNC[] = UBLOX_DEFAULT_SYNC;
static int ublox_lasterror = 0;

// internal prototypes
static void ublox_calculate_checksum(UBloxPacket *packet);
static int ublox_error(int code);

#ifdef _QUEUE_H
int ublox_findInQueue(Queue *q, UBloxHeader *ublox) {
  int offset;
  unsigned short length = 0;

  ublox_lasterror = 0;

  while (qlength(q) > 0) {
    // find Start Delimiter
    if ((offset = qfind(q, UBLOX_SYNC)) == -QUEUE_ERROR_NOTFOUND) {
      // no Start Delimiter => clear queue
      qclear(q);
      return 0;
    } else if (offset > 0) {
      // discard first <offset> bytes from queue
      // printf("uBlox: discarded %d bytes from queue\n", offset);
      qread(q, NULL, offset);
    }

    if (qlength(q) < 8) return 0;
    if (ublox != NULL) qcopy(q, ublox, 4, 2);
    qcopy(q, &length, 2, 4);
    length += 8;
    if (length > UBLOX_MAXLENGTH) {
      qread(q, NULL, 2); // read Start Delimiter and discard package
      continue;
    }
    break;
  }

  if (qlength(q) < length) return 0; // wait until whole packet is received
  return length;
}

int ublox_decodeFromQueue(Queue *q, void *payload, size_t *payloadLength, UBloxHeader *ublox) {
  ssize_t length;
  char *check;
  char checksumA = 0, checksumB = 0;
  int i;
  UBloxHeader ublox_local;
  if (ublox == NULL) ublox = &ublox_local;

  ublox_lasterror = 0;

  if ((length = ublox_findInQueue(q, NULL)) <= 0) return length;

  qread(q, NULL, 2); // sync
  qread(q, ublox, sizeof(*ublox));
  if (ublox.payloadLength > *payloadLength) return ublox_error(UBLOX_ERROR_BIGPACKET);
  
  // checksum calculations
  checksumA += ublox.classId;
  checksumB += checksumA;
  checksumA += ublox.messageId;
  checksumB += checksumA;
  checksumA += (length & 0xFF);
  checksumB += checksumA;
  checksumA += (length >> 8);
  checksumB += checksumA;

  qread(q, payload, ublox.payloadLength);
  for (check = (char *) payload, i = 0; i < ublox.payloadLength; i++) {
    checksumA += check[i];
    checksumB += checksumA;
  }
  if (checksumA != qget(q) || checksumB != qget(q)) return ublox_error(UBLOX_ERROR_CHECKSUM);

  return length;
}
#endif

int ublox_findNext(char **pos, size_t *size) {
	if ((*size >= UBLOX_FRAMESIZE) && ((*pos)[0] == UBLOX_SYNC[0]) && ((*pos)[1] == UBLOX_SYNC[1])) {
		uint16_t length = (uint8_t) (*pos)[4] + ((uint8_t) (*pos)[5] * 256) + UBLOX_FRAMESIZE;
		if (length > *size) length = *size;
		*pos += length;
		*size -= length;
	}

	return ublox_find(pos, size, NULL);
}

int ublox_find(char **pos, size_t *size, UBloxHeader *ublox) {
        // int i = 0;
	uint16_t payloadLength = 0;

	ublox_lasterror = 0;
	while(1) {
	  while(*size > 1) {
		  if ((*pos)[0] == UBLOX_SYNC[0] && (*pos)[1] == UBLOX_SYNC[1]) break;
		  (*pos)++; (*size)--;
	  }
	  if (*size < UBLOX_FRAMESIZE) return 0;

	  payloadLength = (uint8_t) (*pos)[4] + ((uint8_t) (*pos)[5] * 256);
	  if (ublox != NULL) {
		  ublox->classId = (*pos)[2];
		  ublox->messageId = (*pos)[3];
		  ublox->payloadLength = payloadLength;
	  }

    if (payloadLength > UBLOX_MAXPAYLOAD) {
 		  (*pos)++; (*size)--;
      continue;
    }
    break;
  }

  if (*size < payloadLength + UBLOX_FRAMESIZE) return 0;
	return payloadLength + UBLOX_FRAMESIZE;
}

int ublox_decodePacket(char *buffer, size_t size, UBloxPacket **packet) {
	int ret;
	
	ret = ublox_decode(buffer, size, NULL, NULL, NULL);
	if (ret > 0) {
		*packet = (UBloxPacket *) buffer;
	}
	return ret;
}

int ublox_decode(char *buffer, size_t size, void **payload, size_t *payloadLength, UBloxHeader *ublox) {
	char checksumA = 0, checksumB = 0;
	int i = 0;
	size_t pLength;

	ublox_lasterror = 0;
	if (size < 8) return 0;

	if (buffer[i++] != UBLOX_SYNC[0]) return ublox_error(UBLOX_ERROR_NOUBLOX);
	if (buffer[i++] != UBLOX_SYNC[1]) return ublox_error(UBLOX_ERROR_NOUBLOX);
	if (ublox != NULL) {
		ublox->classId = buffer[i++];
		ublox->messageId = buffer[i++];
	} else {
	    i+=2;
	}
	pLength  = (uint8_t) buffer[i++];
	pLength |= (uint8_t) buffer[i++] << 8;
	if (ublox != NULL) {
	    ublox->payloadLength = pLength;
	}
  
	if (size < pLength + UBLOX_FRAMESIZE) return 0;
	if (payload != NULL) *payload = &buffer[i];
	if (payloadLength != NULL) *payloadLength = pLength;

	i += pLength;
	checksumA = buffer[i++];
	checksumB = buffer[i++];

	// checksum calculations
	for (i -= 3; i >= 2; i--) {
	    checksumB -= checksumA;
	    checksumA -= buffer[i];
	}
	if (checksumA != 0 || checksumB != 0) return ublox_error(UBLOX_ERROR_CHECKSUM);

	return pLength + UBLOX_FRAMESIZE;
}

int ublox_delete(char *buffer, size_t *size, char *pos) {
    if (pos == buffer) return 0;
    if (pos > buffer + *size) pos = buffer + *size;
	*size -= pos - buffer;
	memmove(buffer, pos, *size);
    return (pos - buffer);
}

int ublox_decodeAttributes(UBloxPacket *packet, size_t offset, UBloxAttribute* attributes, uint8_t attributesMax) {
  int i = 0;
  size_t c = offset;

  ublox_lasterror = 0;

  while (i < attributesMax && c < packet->payloadLength) {
    attributes[i].type = *(packet->payload + c);
    attributes[i].length = *(packet->payload + c + 1);
    attributes[i].value = (void *) (packet->payload + c + 2);
    c += attributes[i].length + 2;
    i++;
  }

  return (c == packet->payloadLength ? i : -1);
}

int ublox_encodeAttributes(UBloxPacket *packet, size_t offset, UBloxAttribute* attributes, uint8_t attributesCount) {
  int i;
  size_t length = 0;
  char *start = packet->payload + offset;

  ublox_lasterror = 0;

  for (i = 0; i < attributesCount; i++) {
    if (UBLOX_MAXPAYLOAD - offset - (length + attributes->length + 2) < 0) return 0;
    *(start++) = attributes[i].type;
    *(start++) = attributes[i].length;
    memcpy(start, attributes[i].value, attributes[i].length);
    start += attributes[i].length;
    length += attributes[i].length + 2;
  }

  return length;
}

void ublox_calculate_checksum(UBloxPacket *packet) {
  char checksumA = 0, checksumB = 0;
  int i;

  ublox_lasterror = 0;

  for (i = 2; i < packet->payloadLength + 6; i++) {
    checksumA = checksumA + *((char *) packet + i);
    checksumB += checksumA;
  }

  // packet->checksumA = checksumA;
  // packet->checksumB = checksumB;
  *(packet->payload + packet->payloadLength + 0) = checksumA;
  *(packet->payload + packet->payloadLength + 1) = checksumB;
}

int ublox_sendPacket(UBloxPacket *packet, ublox_callback_function func) {
  ublox_lasterror = 0;

  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  ublox_calculate_checksum(packet);
  return func((char *) packet, packet->payloadLength + UBLOX_FRAMESIZE);
}

int ublox_send(const void *message, size_t length, uint8_t classId, uint8_t messageId, ublox_callback_function func) {
  UBloxPacket packet;

  ublox_lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD) return ublox_error(UBLOX_ERROR_BIGPACKET);

  packet.classId = classId;
  packet.messageId = messageId;
  packet.payloadLength = length;
  memcpy(packet.payload, message, length);
  return ublox_sendPacket(&packet, func);
}

#ifdef _QUEUE_H
int ublox_sendToQueue(const void *message, size_t length, uint8_t classId, uint8_t messageId, Queue *q) {
  UBloxPacket packet;

  ublox_lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD) return ublox_error(UBLOX_ERROR_BIGPACKET);

  packet.sync1 = UBLOX_SYNC[0];
  packet.sync2 = UBLOX_SYNC[1];
  packet.classId = classId;
  packet.messageId = messageId;
  packet.payloadLength = length;
  memcpy(packet.payload, message, length);
  ublox_calculate_checksum(&packet);

  return qwrite(q, &packet, length + UBLOX_FRAMESIZE);
}
#endif

int ublox_encode(const void *message, size_t length, uint8_t classId, uint8_t messageId, char *buffer, size_t size) {
  UBloxPacket *packet = (UBloxPacket *) buffer;

  ublox_lasterror = 0;
  if (length > UBLOX_MAXPAYLOAD || size < length + 8) return ublox_error(UBLOX_ERROR_BIGPACKET);

  if (message != NULL) memcpy(packet->payload, message, length);
  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  packet->classId = classId;
  packet->messageId = messageId;
  packet->payloadLength = length;

  ublox_calculate_checksum(packet);
  return length + UBLOX_FRAMESIZE;
}

int ublox_error(int c) {
	ublox_lasterror = c;
	return -c;
}

const char *ublox_strlasterror(void) {
	return ublox_strerror(ublox_lasterror);
}

const char *ublox_strerror(int err) {
    const char *errstr[] = { "",
        "wrong checksum", // UBLOX_ERROR_CHECKSUM
        "illegal packet size", // UBLOX_ERROR_BIGPACKET
        "invalid uBlox frame", // UBLOX_ERROR_NOUBLOX
    };
    if (-err < 0 || ((size_t) -err) > sizeof(errstr)/sizeof(*errstr)) return "";
    return errstr[-err];
}
