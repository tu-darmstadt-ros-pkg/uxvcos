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
#include <stdio.h>

#include "stream/ubx/Protocol.h"

namespace UBX {

#ifdef STREAM_STREAM_H
/*
int Protocol::decode(RingBuffer<unsigned char> &buffer, void *payload, unsigned *payloadLength, Header *header)
{
  unsigned char *check;
  unsigned char checksumA = 0, checksumB = 0;
  int i;
  Header header_local;
  if (header == NULL) header = &header_local;

  lastError = UBLOX_NO_ERROR;

  if (buffer.find(UBLOX_SYNC, sizeof(UBLOX_SYNC)) != 0) return setLastError(UBLOX_ERROR_NOUBLOX);
  if (buffer.length() < 8) return setLastError(UBLOX_ERROR_NOUBLOX);

  buffer.copy((unsigned char *) header, sizeof(*header), 2);
  if (buffer.length() < (unsigned)(header->payloadLength + UBLOX_FRAMESIZE)) return setLastError(UBLOX_ERROR_NOUBLOX);
  if (header->payloadLength > *payloadLength) return setLastError(UBLOX_ERROR_BIGPACKET);
  *payloadLength = header->payloadLength;

  buffer.read(NULL, 2 + sizeof(*header));       // sync + header

  // checksum calculations
  checksumA += header->classId;
  checksumB += checksumA;
  checksumA += header->messageId;
  checksumB += checksumA;
  checksumA += (header->payloadLength & 0xFF);
  checksumB += checksumA;
  checksumA += (header->payloadLength >> 8);
  checksumB += checksumA;

  buffer.read((unsigned char *) payload, *payloadLength);
  for (check = (unsigned char *) payload, i = 0; i < (int) *payloadLength; i++)
  {
    checksumA += check[i];
    checksumB += checksumA;
  }
  if (checksumA != buffer.get() || checksumB != buffer.get()) return setLastError(UBLOX_ERROR_CHECKSUM);

  return *payloadLength + UBLOX_FRAMESIZE;
}
*/
#endif // STREAM_STREAM_H

int Protocol::findNext(unsigned char **pos, unsigned *size)
{
  if ((*size >= UBLOX_FRAMESIZE) && ((*pos)[0] == UBLOX_SYNC[0]) && ((*pos)[1] == UBLOX_SYNC[1]))
  {
    uint16_t length = (unsigned char)(*pos)[4] + ((unsigned char)(*pos)[5] * 256) + UBLOX_FRAMESIZE;
    if (length > *size) length = *size;
    *pos += length;
    *size -= length;
  }

  return find(pos, size, NULL);
}

int Protocol::find(unsigned char **pos, unsigned *size, Header *header)
{
  uint16_t payloadLength = 0;

  lastError = UBLOX_NO_ERROR;
  while (*size > 1)
  {
    if ((*pos)[0] == UBLOX_SYNC[0] && (*pos)[1] == UBLOX_SYNC[1]) break;
    (*pos) ++; (*size)--;
  }
  if (*size < UBLOX_FRAMESIZE) return 0;

  payloadLength = (unsigned char)(*pos)[4] + ((unsigned char)(*pos)[5] * 256);
  if (header != NULL)
  {
    header->classId = (*pos)[2];
    header->messageId = (*pos)[3];
    header->payloadLength = payloadLength;
  }

  return payloadLength + UBLOX_FRAMESIZE;
}

int Protocol::decodePacket(unsigned char *buffer, unsigned size, Packet **packet)
{
  int ret;

  ret = decode(buffer, size, NULL, NULL, NULL);
  if (ret > 0)
  {
    *packet = (Packet *) buffer;
  }
  return ret;
}

int Protocol::decode(unsigned char *buffer, unsigned size, void **payload, unsigned *payloadLength, Header *header)
{
  unsigned char checksumA = 0, checksumB = 0;
  int i = 0;
  unsigned pLength;

  lastError = UBLOX_NO_ERROR;
  if (size < 8) return 0;

  if (buffer[i++] != UBLOX_SYNC[0]) return setLastError(UBLOX_ERROR_NOUBLOX);
  if (buffer[i++] != UBLOX_SYNC[1]) return setLastError(UBLOX_ERROR_NOUBLOX);
  if (header != NULL)
  {
    header->classId = buffer[i++];
    header->messageId = buffer[i++];
  }
  else
  {
    i += 2;
  }
  pLength  = (unsigned char) buffer[i++];
  pLength |= (unsigned char) buffer[i++] << 8;
  if (header != NULL)
  {
    header->payloadLength = pLength;
  }

  if (size < pLength + UBLOX_FRAMESIZE) return 0;
  if (payload != NULL) *payload = &buffer[i];
  if (payloadLength != NULL) *payloadLength = pLength;

  i += pLength;
  checksumA = buffer[i++];
  checksumB = buffer[i++];

  // checksum calculations
  for (i -= 3; i >= 2; i--)
  {
    checksumB -= checksumA;
    checksumA -= buffer[i];
  }
  if (checksumA != 0 || checksumB != 0) return setLastError(UBLOX_ERROR_CHECKSUM);

  return pLength + UBLOX_FRAMESIZE;
}

int Protocol::remove(unsigned char *buffer, unsigned *size, unsigned char *pos)
{
  if (pos == buffer) return 0;
  if (pos > buffer + *size) pos = buffer + *size;
  *size -= pos - buffer;
  memmove(buffer, pos, *size);
  return (pos - buffer);
}

int Protocol::decodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, unsigned char attributesMax)
{
  int i = 0;
  unsigned c = offset;

  lastError = UBLOX_NO_ERROR;

  while (i < attributesMax && c < packet->payloadLength)
  {
    attributes[i].type = * (packet->payload + c);
    attributes[i].length = * (packet->payload + c + 1);
    attributes[i].value = (void *)(packet->payload + c + 2);
    c += attributes[i].length + 2;
    i++;
  }

  return (c == packet->payloadLength ? i : -1);
}

int Protocol::encodeAttributes(Packet *packet, unsigned offset, Attribute* attributes, unsigned char attributesCount)
{
  int i;
  unsigned length = 0;
  unsigned char *start = packet->payload + offset;

  lastError = UBLOX_NO_ERROR;

  for (i = 0; i < attributesCount; i++)
  {
    if (offset + length + attributes->length + 2 > UBLOX_MAXPAYLOAD) return 0;
    * (start++) = attributes[i].type;
    * (start++) = attributes[i].length;
    memcpy(start, attributes[i].value, attributes[i].length);
    start += attributes[i].length;
    length += attributes[i].length + 2;
  }

  return length;
}

void Protocol::calculate_checksum(Packet *packet)
{
  Checksum checksum = Protocol::checksum(reinterpret_cast<Header *>(&(packet->sync2) + 1), packet->payload, packet->payloadLength);
  *(packet->payload + packet->payloadLength + 0) = checksum.checksumA;
  *(packet->payload + packet->payloadLength + 1) = checksum.checksumB;
}

Checksum Protocol::checksum(const Header *header, const unsigned char *message, unsigned length) {
  Checksum checksum;
  checksum.add(reinterpret_cast<const unsigned char *>(header), sizeof(*header));
  checksum.add(message, length);
  return checksum;
}

int Protocol::sendPacket(Packet *packet, callback_function func)
{
  lastError = UBLOX_NO_ERROR;

  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  calculate_checksum(packet);
  return func((unsigned char *) packet, packet->payloadLength + UBLOX_FRAMESIZE);
}

int Protocol::send(const void *message, unsigned length, unsigned char classId, unsigned char messageId, callback_function func)
{
  Packet packet;

  lastError = UBLOX_NO_ERROR;
  if (length > UBLOX_MAXPAYLOAD) return setLastError(UBLOX_ERROR_BIGPACKET);

  packet.classId = classId;
  packet.messageId = messageId;
  packet.payloadLength = length;
  memcpy(packet.payload, message, length);
  return sendPacket(&packet, func);
}

int Protocol::encode(const void *message, const unsigned length, const unsigned char classId, const unsigned char messageId, unsigned char *buffer, const unsigned size)
{
  Packet *packet = (Packet *) buffer;

  lastError = UBLOX_NO_ERROR;
  if (length > UBLOX_MAXPAYLOAD || size < length + 8) return setLastError(UBLOX_ERROR_BIGPACKET);

  if (message != NULL) memcpy(packet->payload, message, length);
  packet->sync1 = UBLOX_SYNC[0];
  packet->sync2 = UBLOX_SYNC[1];
  packet->classId = classId;
  packet->messageId = messageId;
  packet->payloadLength = length;

  calculate_checksum(packet);
  return length + UBLOX_FRAMESIZE;
}

bool Protocol::setSync(const std::string& sync) {
  unsigned short temp[2];
  if (sscanf(sync.c_str(), "%2hx%2hx", &temp[0], &temp[1]) != 2) return false;
  UBLOX_SYNC[0] = (unsigned char) temp[0];
  UBLOX_SYNC[1] = (unsigned char) temp[1];
  return true;
}


} // namespace UBX
