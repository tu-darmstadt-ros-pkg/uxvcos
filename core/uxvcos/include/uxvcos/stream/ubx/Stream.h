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

#ifndef STREAM_UBX_STREAM_H
#define STREAM_UBX_STREAM_H

#include "Protocol.h"

#include <uxvcos/system/systemcalls.h> // for sscanf
#include <uxvcos/data/Streamable.h>
#include <uxvcos/Time.h>
#include <uxvcos/stream/Stream.h>
#include <uxvcos/stream/Buffer.h>

#include <uxvcos/uxvcos.h>

namespace UBX {

class UXVCOS_API Decoder : public InStream {
public:
  Decoder(InStream* source)
    : source(source)
    , state(START)
  {
  }

  virtual ~Decoder() {
  }

  bool find();
  bool verify_checksum();

  virtual size_t readsome(void *destination, size_t n);
  virtual InStream& read(void *destination, size_t n);

  template<typename T> bool read(T& target) { return read(&target, sizeof(target)) && verify_checksum(); }
  bool read(Data::Streamable& data);
  Data::Streamable* read(void *place = 0);

  template<typename T>
  InStream& operator>>(T& target) {
    this->error(false);
    if (!read(target)) this->error(true);
    return *this;
  }

  template<typename T>
  InStream& operator>>(T* target) {
    this->error(false);
    if (!read(target)) this->error(true);
    return *this;
  }

  virtual size_t size() const {
    return remaining;
  }

  virtual InStream& start(size_t size = 0);
  virtual InStream& finish();

  Protocol* ublox() { return &ubx; }
  Header* header() { return &h; }
  virtual const char *strerror(const int err) const { return ubx.strError((UBX::Error) err); }
  
  uxvcos::Time getTimestamp() const { return currentTimestamp; }
  void setTimestamp(uxvcos::Time t) { currentTimestamp = t; }

  // ROS compatibility methods
  virtual element_t *data();
  virtual element_t *del(ssize_t);

protected:
  Protocol ubx; // TODO: pointer to protocol implementation (to reduce dependencies)
  InStream* source;
  unsigned char buffer[Data::Streamable::MAXSIZE];
  uxvcos::Time currentTimestamp;

private:
  enum { START, SYNC1, SYNC2, HEADER, DATA } state;
  Header h;
  Checksum checksum;
  unsigned int remaining;
};


class UXVCOS_API Encoder : public OutStream {
public:
  Encoder(OutStream* target)
    : target(target)
    , buffer(UBX::UBLOX_MAXLENGTH)
  {
  }

  virtual ~Encoder() {
  }

  Header* addHeader(uint8_t classId = 0, uint8_t messageId = 0, unsigned short payloadLength = 0);
  bool addChecksum(Header* header);
  bool add(const void *message, unsigned length, uint8_t classId, uint8_t messageId);
  bool write(const void *message, unsigned length, uint8_t classId, uint8_t messageId);
  bool add(const Data::Streamable& data);
  bool write(const Data::Streamable& data);
  bool write();

  Protocol* ublox() { return &ubx; }
  virtual const char *strerror(const int err) const { return Protocol::strError((UBX::Error) err); }

protected:
  Protocol ubx; // TODO: pointer to protocol implementation (to reduce dependencies)
  OutStream* target;
  Buffer<unsigned char> buffer;
  uxvcos::Time currentTimestamp;
};

} // namespace UBX

#endif // STREAM_UBX_STREAM_H
