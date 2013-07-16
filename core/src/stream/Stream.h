//!  InStream, OutStream and InOutStream Classes
/*!
  These classes provides templates for handling streams.
*/

#ifndef STREAM_STREAM_H
#define STREAM_STREAM_H

#include <limits.h>
#include <boost/array.hpp>
#include <vector>
#include <map>
#include <string>

#include "BaseStream.h"

#ifdef ROS_PACKAGE_NAME
  #include <ros/serialization.h>
#endif

class UXVCOS_API InStream : public virtual BaseStream
{
public:
  static const element_t EoF = 0;

  InStream() : _eof(false) {}
  virtual ~InStream() {}

  virtual element_t get() {
    return EoF;
  }

  virtual size_t readsome(void *destination, size_t n) {
    _eof = true;
    return 0;
  }

  virtual size_t size() const {
    return 0;
  }

  virtual InStream& read(void *destination, size_t n)
  {
    _eof = !(size() >= n && readsome(destination, n) == n);
    return *this;
  }

  virtual bool find(const element_t& delim)
  {
    element_t res;

    do {
      res = get();
    } while (!eof() && res != delim);

    if (res == delim) return true;
    return false;
  }

  virtual bool find(const element_t delim[], size_t n)
  {
    index_t j = 0;
    element_t res;

    while(j < n) {
      res = get();
      if (eof()) break;
      if (res != delim[j])
        j = 0;
      else
        j++;
    }

    if (j == n) return true;
    return false;
  }

  virtual bool wait(unsigned long msecs) {
    return true;
  }

  virtual InStream& start(size_t size = 0) {
    return *this;
  }

  virtual InStream& finish() {
    return *this;
  }

  template <typename T>
  InStream& operator&(T &value) {
    return this->operator>>(value);
  }

  virtual InStream& operator>>(float &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(double &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(bool &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(signed char &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(unsigned char &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(signed short &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(unsigned short &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(signed int &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(unsigned int &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(signed long &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(unsigned long &value) {
    return read(&value, sizeof(value));
  }

  virtual InStream& operator>>(std::vector<unsigned char> &value) {
    unsigned char c;

    value.clear();
    value.reserve(size());
    while(read(&c, 1)) value.push_back(c);
    return *this;
  }

  virtual InStream& operator>>(Data::Streamable &data) {
    error(!read(data));
    return *this;
  }

  virtual bool read(Data::Streamable &data)
  {
    return false;
  }

  virtual bool eof() const {
    return _eof;
  }

  virtual bool good() const {
    return BaseStream::good() && !eof();
  }

  virtual void reset() {
    BaseStream::reset();
    _eof = false;
  }

  virtual element_t *data() { return 0; }
  virtual const element_t *data() const { return const_cast<InStream *>(this)->data(); }
  virtual element_t *del(ssize_t len) { return 0; }

  // ROS compatibility methods
  virtual element_t *getData() {
    return data();
  }

  virtual size_t getLength() {
    return size();
  }

  virtual element_t *advance(size_t len) {
    element_t *data = del(len);
#ifdef ROSCPP_SERIALIZATION_H
    if (!data) ros::serialization::throwStreamOverrun();
#endif
    return data;
  }

#ifdef ROSCPP_SERIALIZATION_H
  template <typename T>
  InStream& next(T &value) {
    try {
      ros::serialization::deserialize(*this, value);
    } catch(ros::serialization::StreamOverrunException&) {
      fail();
    }
//    ros::serialization::IStream istream(getData(), getLength());
//    try {
//      ros::serialization::deserialize(istream, value);
//    } catch (ros::StreamOverrunException& e) {

//    advance(istream.getData() - getData());
    return *this;
  }
#endif

protected:
  bool _eof;
};

class UXVCOS_API OutStream : public virtual BaseStream
{
public:
  OutStream() : _overflow(false) {}
  virtual ~OutStream() {}

  virtual OutStream& write(const void *source, size_t size) {
    _overflow = true;
    return *this;
  }

  virtual size_t free() const {
    return UINT_MAX;
  }

  virtual OutStream& put(unsigned char c) {
    return write(&c, 1);
  }

  virtual OutStream& start(size_t size = 0) {
    _overflow = false;
    if (size != 0 && size > free()) _overflow = true;
    return *this;
  }

  virtual OutStream& finish() {
    return *this;
  }

  virtual void flush() {
  }

  template <typename T>
  OutStream& operator&(const T &value) {
    return this->operator<<(value);
  }

  virtual OutStream& operator<<(float value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(double value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(bool value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(signed char value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(unsigned char value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(signed short value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(unsigned short value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(signed int value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(unsigned int value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(signed long value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(unsigned long value) {
    return write(&value, sizeof(value));
  }

  virtual OutStream& operator<<(const std::string& s) {
    return write(s.data(), s.length());
  }

  virtual OutStream& operator<<(const std::vector<unsigned char> &value) {
    return write(&value[0], value.size() * sizeof(value[0]));
  }

  virtual OutStream& operator<<(const Data::Streamable& source) {
    _overflow = !write(source);
    return *this;
  }

  virtual bool write(const Data::Streamable& source) {
    return false;
  }

  virtual bool overflow() const {
    return _overflow;
  }

  virtual bool good() const {
    return BaseStream::good() && !overflow();
  }

  virtual void reset() {
    BaseStream::reset();
    _overflow = false;
  }

  virtual element_t *tail() { return 0; }
  virtual const element_t *tail() const { return const_cast<OutStream *>(this)->tail(); }
  virtual element_t *add(ssize_t len) { return 0; }

  // ROS compatibility methods
  virtual element_t *getData() {
    return tail();
  }

  virtual size_t getLength() {
    return free();
  }

  virtual element_t *advance(size_t len) {
    element_t *data = add(len);
#ifdef ROSCPP_SERIALIZATION_H
    if (!data) ros::serialization::throwStreamOverrun();
#endif
    return data;
  }

#ifdef ROSCPP_SERIALIZATION_H
  template <typename T>
  OutStream& next(const T &value) {
    try {
      ros::serialization::serialize(*this, value);
    } catch(ros::serialization::StreamOverrunException&) {
      fail();
    }
//    ros::serialization::OStream ostream(getData(), ros::serialization::serializationLength(value));
//    if (!start(ostream.getLength())) return *this;
//    ros::serialization::serialize(ostream, value);
//    advance(ostream.getData() - getData());
//    finish();
    return *this;
  }
#endif

protected:
  bool _overflow;
};

class UXVCOS_API Stream : public InStream, public OutStream
{
public:
  virtual bool good() const {
    return InStream::good() && OutStream::good();
  }

  virtual void reset() {
    InStream::reset();
    OutStream::reset();
  }

  template <typename T>
  BaseStream& operator&(const T &value) {
    return *this;
  }

  virtual Stream& start(size_t size = 0) {
    return *this;
  }

  virtual Stream& finish() {
    return *this;
  }
};

template<class T, std::size_t N>
static inline InStream& operator>>(InStream& in, boost::array<T,N> &array) {
  for(size_t i = 0; i < array.size(); ++i) in >> array[i];
  return in;
}

template<class T, std::size_t N>
static inline OutStream& operator<<(OutStream& out, const boost::array<T,N> &array) {
  for(size_t i = 0; i < array.size(); ++i) out << array[i];
  return out;
}

// read as many data as possible!
template<class T>
static inline InStream& operator>>(InStream& in, std::vector<T> &vector) {
  T value;
  vector.clear();
  while(in >> value) vector.push_back(value);
  in.reset();
  return in;
}

template<class T>
static inline OutStream& operator<<(OutStream& out, const std::vector<T> &vector) {
  for(std::size_t i = 0; i < vector.size(); ++i) out << vector[i];
  return out;
}

template<class Key, class T>
static inline InStream& operator>>(InStream& in, std::map<Key,T> &map) {
  Key key; T value; std::size_t n;
  map.clear();
  if (in >> n) {
    while(n-- > 0 && in.good()) {
      in >> key >> value;
      map[key] = value;
    }
  }
  return in;
}

template<class Key, class T>
static inline OutStream& operator<<(OutStream& out, const std::map<Key,T> &map) {
  out << map.size();
  for(typename std::map<Key,T>::const_iterator it = map.begin(); it != map.end(); ++it)
    out << it->first << it->second;
  return out;
}

#ifdef OROCOS_TARGET
  #include <rtt/base/DataSourceBase.hpp>
  UXVCOS_API OutStream& operator<<(OutStream& out, const RTT::base::DataSourceBase::shared_ptr dsb);
  UXVCOS_API InStream& operator>>(InStream& in, RTT::base::DataSourceBase::shared_ptr dsb);
#endif // OROCOS_TARGET

#endif // STREAM_STREAM_H
