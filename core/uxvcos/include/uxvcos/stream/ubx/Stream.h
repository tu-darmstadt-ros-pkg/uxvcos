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
