#ifndef DATA_STREAMABLEFORMATTER_H
#define DATA_STREAMABLEFORMATTER_H

#include <boost/format.hpp>
#include <vector>

namespace Data {

class StreamableFormatter : public boost::format {
public:
  StreamableFormatter(const std::string &format) : boost::format(format) { exceptions(0); }
  StreamableFormatter(const char *format) : boost::format(format) { exceptions(0); }

  template <typename T> StreamableFormatter& operator&(const T &value) {
    return print(value);
  }

  template <typename T> StreamableFormatter& print(const T &value);
  template <typename T> StreamableFormatter& print(const T *value);
  template <typename T> StreamableFormatter& print(const std::vector<T> &value);

  void fail() {}
};

// general format function
template <typename T>
StreamableFormatter& StreamableFormatter::print(const T &value) {
  *this % value;
  return *this;
}

// specialization for pointers
template <typename T>
StreamableFormatter& StreamableFormatter::print(const T *value) {
  *this % *value;
  return *this;
}

// specialization for vector types
template <typename T>
StreamableFormatter& StreamableFormatter::print(const std::vector<T> &value) {
  for (typename std::vector<T>::const_iterator it = value.begin(); it != value.end(); ++it)
    *this % (*it);
  return *this;
}

} // namespace Data

#endif // DATA_STREAMABLEFORMATTER_H
