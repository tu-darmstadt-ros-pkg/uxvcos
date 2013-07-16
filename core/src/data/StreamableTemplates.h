#ifndef DATA_STREAMABLETEMPLATES_H
#define DATA_STREAMABLETEMPLATES_H

#include "Streamable.h"
#include <vector>
#include <boost/array.hpp>

namespace Data {

template<typename T>
struct UXVCOS_API StreamableValue : public Streamable {
  T value;

  StreamableValue(const T& init = T()) : value(init) {}
  StreamableValue(const StreamableValue<T>& other) : Streamable(other), value(other.value) {}
  virtual ~StreamableValue() {}

  operator T() { return value; }
  T& operator=(const T& other) { value = other; }

  InStream& operator<<(InStream& in)
  {
    return in >> value;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << value;
  }
};

template<typename T, size_t N = 1>
struct UXVCOS_API StreamableArray : public Streamable {
  typedef boost::array<T,N> Array;
  Array value;

  StreamableArray(const T& init = T()) {
    value.assign(init);
  }
  StreamableArray(const StreamableArray<T,N>& other) : Streamable(other) {
    value = other.value;
  }
  virtual ~StreamableArray() {}

  operator Array() { return value; }
  Array& operator=(const Array& other) { value = other; }
  T& operator[](size_t i) { return value[i]; }
  const T& operator[](size_t i) const { return value[i]; }

  InStream& operator<<(InStream& in)
  {
    return in >> value;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << value;
  }
};

template <typename T>
struct UXVCOS_API StreamableVector : public Streamable
{
  typedef std::vector<T> Vector;
  Vector vector;

  StreamableVector(size_t n = 0, const T& init = T()) : vector(n, init) {}
  StreamableVector(const Vector& other) : vector(other) {}
  StreamableVector(const StreamableVector<T>& other) : Streamable(other), vector(other.vector) {}
  virtual ~StreamableVector() {}

  operator Vector() { return vector; }
  Vector& operator=(const Vector& other) { vector = other; }
  T& operator[](size_t i) { return vector[i]; }
  const T& operator[](size_t i) const { return vector[i]; }

  InStream& operator<<(InStream& in)
  {
    return in >> vector;
  }

  OutStream& operator>>(OutStream& out) const
  {
    return out << vector;
  }
};

} // namespace Data

#endif // DATA_STREAMABLETEMPLATES_H
