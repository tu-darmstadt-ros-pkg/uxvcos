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
