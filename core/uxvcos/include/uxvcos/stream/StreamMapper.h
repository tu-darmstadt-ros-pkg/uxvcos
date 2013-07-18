#ifndef STREAM_STREAMMAPPER_H
#define STREAM_STREAMMAPPER_H

#include <stream/Stream.h>
#include <iostream>

class InStreamMapper : public InStream {
public:
  InStreamMapper(std::istream& is) : stream(is) {}
  virtual ~InStreamMapper() {}

  virtual element_t get() {
    return static_cast<element_t>(stream.get());
  }

  virtual size_t readsome(void *destination, size_t n) {
    return stream.readsome(reinterpret_cast<char *>(destination), n);
  }

  virtual InStream& read(void *destination, size_t n)
  {
    stream.read(reinterpret_cast<char*>(destination), n);
    return *this;
  }

  virtual InStream& operator>>(float &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(double &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(bool &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(signed char &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(unsigned char &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(signed short &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(unsigned short &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(signed int &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(size_t &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(signed long &value) {
    stream >> value;
    return *this;
  }

  virtual InStream& operator>>(unsigned long &value) {
    stream >> value;
    return *this;
  }

  virtual bool eof() const {
    return stream.eof();
  }

  virtual bool good() const {
    return BaseStream::good() && stream.good();
  }

private:
  std::istream& stream;
};


class OutStreamMapper : public OutStream {
public:
  OutStreamMapper(std::ostream& os) : stream(os) {}

  virtual OutStream& write(const void *source, size_t size) {
    stream.write(reinterpret_cast<const char *>(source), size);
    return *this;
  }

  virtual OutStream& put(unsigned char c) {
    stream.put(c);
    return *this;
  }

  virtual void flush() {
    stream.flush();
  }

  virtual OutStream& operator<<(float value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(double value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(bool value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(signed char value) {
    stream << ' ' << static_cast<signed int>(value);
    return *this;
  }

  virtual OutStream& operator<<(unsigned char value) {
    stream << ' ' << static_cast<unsigned int>(value);
    return *this;
  }

  virtual OutStream& operator<<(signed short value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(unsigned short value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(signed int value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(size_t value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(signed long value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(unsigned long value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(const std::string &value) {
    stream << ' ' << value;
    return *this;
  }

  virtual OutStream& operator<<(const std::vector<unsigned char> &value) {
    std::vector<unsigned char>::const_iterator iter;
    stream << ' ';
    for(iter = value.begin(); iter != value.end(); ++iter)
      stream << *iter;
    return *this;
  }

  template<class T, std::size_t N>
  friend OutStreamMapper& operator<<(OutStreamMapper& out, const boost::array<T,N> &array);

  template<class T>
  friend OutStreamMapper& operator<<(OutStreamMapper& out, const std::vector<T> &vector);

  template<class Key, class T>
  friend OutStreamMapper& operator<<(OutStreamMapper& out, const typename std::map<Key,T> &map);

  virtual bool overflow() const {
    return stream.eof();
  }

  virtual bool good() const {
    return BaseStream::good() && stream.good();
  }

private:
  std::ostream& stream;
};

template <typename Archive>
class InArchiveMapper : public InStream {
public:
  InArchiveMapper(Archive& ar) : archive(ar) {}
  virtual ~InArchiveMapper() {}

  virtual InStream& operator>>(float &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(double &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(bool &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(signed char &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(unsigned char &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(signed short &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(unsigned short &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(signed int &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(size_t &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(signed long &value) {
    archive & value;
    return *this;
  }

  virtual InStream& operator>>(unsigned long &value) {
    archive & value;
    return *this;
  }

private:
  Archive& archive;
};

template <typename Archive>
class OutArchiveMapper : public OutStream {
public:
  OutArchiveMapper(Archive& ar) : archive(ar) {}

  virtual OutStream& operator<<(float value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(double value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(bool value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(signed char value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(unsigned char value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(signed short value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(unsigned short value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(signed int value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(size_t value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(signed long value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(unsigned long value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(const std::string &value) {
    archive & value;
    return *this;
  }

  virtual OutStream& operator<<(const std::vector<unsigned char> &value) {
    archive & value;
    return *this;
  }

private:
  Archive& archive;
};

template<class T, std::size_t N>
static inline OutStreamMapper& operator<<(OutStreamMapper& out, const boost::array<T,N> &array) {
  out.stream << '[';
  for(size_t i = 0; i < array.size(); ++i) {
    if (i != 0) out.stream << ',';
    out.stream << array[i];
  }
  out.stream << ']';
  return out;
}

template<class T>
static inline OutStreamMapper& operator<<(OutStreamMapper& out, const std::vector<T> &vector) {
  out.stream << '[';
  for(size_t i = 0; i < vector.size(); ++i) {
    if (i != 0) out.stream << ',';
    out.stream << vector[i];
  }
  out.stream << ']';
  return out;
}

template<class Key, class T>
static inline OutStreamMapper& operator<<(OutStreamMapper& out, const std::map<Key,T> &map) {
  out.stream << "{ ";
  for(typename std::map<Key,T>::const_iterator it = map.begin(); it != map.end(); ++it) {
    if (it != map.begin()) out.stream << ", ";
    out.stream << it->first << ": " << it->second;
  }
  out.stream << " }";
  return out;
}

#endif // STREAM_STREAMMAPPER_H
