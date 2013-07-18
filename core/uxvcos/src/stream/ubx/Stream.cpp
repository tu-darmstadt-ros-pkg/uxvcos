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

#include "stream/ubx/Stream.h"
#include <data/TypeRegistry.h>

#ifdef OROCOS_TARGET
#include <rtt/Logger.hpp>
#endif

namespace UBX {

bool Decoder::find()
{
  reset();

  if (state >= DATA) state = START;
  if (source->size() == 0) return false;

  // find Start Delimiter
  if (state == START) {
    if (!source->find(static_cast<InStream::element_t>(*(ubx.getSync())))) return false;
    state = SYNC1;
  }

  if (state == SYNC1) {
    if (!source->find(static_cast<InStream::element_t>(*(ubx.getSync() + 1)))) return false;
    state = SYNC2;
  }

  if (state == SYNC2) {
    if (!source->read(&h, sizeof(h))) return false;

    if (h.payloadLength > UBLOX_MAXPAYLOAD)
    {
      state = START; // discard package
      error(UBLOX_ERROR_BIGPACKET);
      return false;
    }

    checksum.reset();
    unsigned char *p = reinterpret_cast<unsigned char *>(&h);
    for(size_t i = 0; i < sizeof(h); i++) {
      checksum.checksumA = (unsigned char)(checksum.checksumA + *p++);
      checksum.checksumB = (unsigned char)(checksum.checksumA + checksum.checksumB);
    }

    remaining = h.payloadLength;
    state = HEADER;
  }

  if (state == HEADER) {
    if (source->size() < remaining + 2) return false;

    state = DATA;
  }

  return (state == DATA);
}

InStream& Decoder::read(void *destination, BaseStream::size_t n)
{
  _eof = (!source->good() || state != DATA || remaining < n);
  if (_eof) return *this;

  if (!source->read(destination, n)) {
    _eof = true;
    return *this;
  }

  unsigned char *p = reinterpret_cast<unsigned char *>(destination);
  for(size_t i = 0; i < n; i++) {
    checksum.checksumA = (unsigned char)(checksum.checksumA + *p++);
    checksum.checksumB = (unsigned char)(checksum.checksumA + checksum.checksumB);
  }

  remaining -= n;

  return *this;
}

BaseStream::size_t Decoder::readsome(void *destination, BaseStream::size_t n)
{
  _eof = (!source->good() || state != DATA);
  if (_eof) return 0;

  if (n > remaining) n = remaining;
  int rv = source->readsome(destination, n);

  unsigned char *p = reinterpret_cast<unsigned char *>(destination);
  for(int i = 0; i < rv; i++) {
    checksum.checksumA = (unsigned char)(checksum.checksumA + *p++);
    checksum.checksumB = (unsigned char)(checksum.checksumA + checksum.checksumB);
  }

  remaining -= rv;
  return rv;
}

bool Decoder::verify_checksum() {
  if (remaining != 0) return false;
  Checksum sourceChecksum;
  
  if (!source->read(&sourceChecksum.checksumA, 2)) return false;
  return checksum == sourceChecksum;
}

bool Decoder::read(Data::Streamable& data) {
  Data::Key key = data.getKey();
  
  if (state != DATA) return false;
  if (key != Data::Key(h.classId, h.messageId)) return false;
  
  data << *this;
  if (!currentTimestamp.isZero()) data.setTimestamp(currentTimestamp);

  return this->good() && verify_checksum();
}

Data::Streamable* Decoder::read(void *place) {
  while(find()) {
    Data::Key key = Data::Key(h.classId, h.messageId);
    if (!key) continue;

    // is Timestamp information?
    if (key == Data::Key::Timestamp && remaining == sizeof(uxvcos::Seconds)) {
      uxvcos::Seconds seconds;
      if (!read(&seconds, sizeof(seconds)) || !verify_checksum()) return 0;
      // assert(currentTimestamp < uxvcos::Time(seconds + 1));
      currentTimestamp.fromSec(seconds);
      return Data::Streamable::BREAK;
    }

    const Data::TypeInfo *typeInfo = Data::types()->getTypeInfo(key);
    if (!typeInfo) continue;

    if (!place) place = buffer;
    Data::Streamable* data = typeInfo->create(place);
    if (!currentTimestamp.isZero()) data->setTimestamp(currentTimestamp);
    data->setKey(key);

    if (!read(*data)) {
      // if (place == 0) delete data;
      return Data::Streamable::INVALID;
    }

    return data;
  }

  return 0;
}

InStream& Decoder::start(size_t size)
{
  reset();
  if (!source->good() || state != DATA || remaining < size) fail();
  return *this;
}

InStream& Decoder::finish()
{
  if (!verify_checksum()) fail();
  return *this;
}

BaseStream::element_t *Decoder::data() {
  if (state != DATA) return 0;
  return source->data();
}

BaseStream::element_t *Decoder::del(ssize_t n) {
  if (n < 0) return 0;
  if (!source->good() || state != DATA || (ssize_t) remaining < n) return 0;

  element_t *data = source->del(n);
  if (!data) return 0;

  const unsigned char *p = reinterpret_cast<const unsigned char *>(data);
  for(int i = 0; i < n; i++) {
    checksum.checksumA = (unsigned char)(checksum.checksumA + *p++);
    checksum.checksumB = (unsigned char)(checksum.checksumA + checksum.checksumB);
  }

  remaining -= n;
  return data;
}

Header* Encoder::addHeader(uint8_t classId, uint8_t messageId, unsigned short payloadLength)
{
  UBX::Header *header;
  UBX::Checksum checksum;

  if (!buffer.write(ubx.getSync(), 2)) return 0;
  
  header = reinterpret_cast<UBX::Header *>(buffer.tail());
  if (!buffer.add(sizeof(*header))) return 0;

  header->classId = classId;
  header->messageId = messageId;
  header->payloadLength = payloadLength;

  return header;
}

bool Encoder::addChecksum(Header* header) {
  UBX::Checksum checksum;
  const void *message = reinterpret_cast<const unsigned char *>(header) + sizeof(*header);

  checksum.add(reinterpret_cast<const unsigned char *>(header), sizeof(*header));
  checksum.add(reinterpret_cast<const unsigned char *>(message), header->payloadLength);
  
  if(!buffer.write(&checksum, sizeof(checksum))) return false;
  return true;
}

bool Encoder::add(const void *message, unsigned length, uint8_t classId, uint8_t messageId)
{
  if (length > UBLOX_MAXPAYLOAD) {
    error(UBLOX_ERROR_BIGPACKET);
    return false;
  }

  UBX::Header* header = addHeader(classId, messageId, static_cast<unsigned short>(length));
  if (!header) return false;
  if (!buffer.write(message, length)) return false;
  if (!addChecksum(header)) return false;
  return true;
}

bool Encoder::write(const void *message, unsigned length, uint8_t classId, uint8_t messageId)
{
  Buffer<unsigned char>::MutexLock lock(&buffer);
  reset();
  return add(message, length, classId, messageId) && write();
}

bool Encoder::add(const Data::Streamable& data) {
  // const Data::TypeInfo *typeInfo = data.getTypeInfo();
  const Data::Key key = data.getKey();
  const uxvcos::Time dataTimestamp = data.getTimestamp();
  // if (!typeInfo) return false;

  if (!key) {
#ifdef OROCOS_TARGET
    RTT::log(RTT::Debug) << "[Encoder::write] Cannot encode data without a key!" << RTT::endlog();
#endif
    return false;
  }

  if (!dataTimestamp.isZero() && (dataTimestamp != currentTimestamp)) {
    currentTimestamp = dataTimestamp;
    uxvcos::Seconds seconds = currentTimestamp.toSec();
    write(&seconds, sizeof(seconds), Data::Key::Timestamp.classId(), Data::Key::Timestamp.messageId());
  }

  UBX::Header *header = addHeader(key.classId(), key.messageId());
  if (!header) return false;
  unsigned int sizeBefore = buffer.size();

  if (!(data >> buffer)) {
#ifdef OROCOS_TARGET
    RTT::log(RTT::Error) << "[Encoder::write] Buffer overflow!" << RTT::endlog();
#endif
    return false;
  }
  
  header->payloadLength = (unsigned short) (buffer.size() - sizeBefore);
  
  if (header->payloadLength > UBLOX_MAXPAYLOAD) {
    error(UBLOX_ERROR_BIGPACKET);
#ifdef OROCOS_TARGET
    RTT::log(RTT::Debug) << "[Encoder::write] payload too big: " << header->payloadLength << RTT::endlog();
#endif
    return false;
  }

  if (!addChecksum(header)) return false;
  return true;
}

bool Encoder::write(const Data::Streamable& data) {
  Buffer<unsigned char>::MutexLock lock(&buffer);
  reset();
  return add(data) && write();
}

bool Encoder::write() {
  Buffer<unsigned char>::MutexLock lock(&buffer);
  size_t size = buffer.size();
  if (target->start(size)) {
    if (!target->write(buffer.data(), size)) return false;
    if (!target->finish()) return false;
  }
  buffer.clear();
  return true;
}

} // namespace UBX
