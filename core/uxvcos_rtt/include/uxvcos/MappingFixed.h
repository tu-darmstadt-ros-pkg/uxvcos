#ifndef UXVCOS_MAPPING_H
#define UXVCOS_MAPPING_H

#include <rtt/TaskContext.hpp>
#include <vector>

namespace uxvcos {

class MappingBase {
public:
  virtual size_t& operator[](size_t index) = 0;
  virtual size_t operator[](size_t index) const = 0;
  virtual size_t size() const = 0;

  std::string toString() const;
  bool fromString(const std::string& s);
};

template<size_t n>
class Mapping : public MappingBase {
public:
  Mapping()
    : map(n)
  {
    for(size_t i = 0; i < n; i++) map[i] = i;
  }

  Mapping<n>& operator=(const Mapping<n> &other) {
    this->map = other.map;
    return *this;
  }

  size_t& operator[](size_t index) {
    return map[index];
  }

  size_t operator[](size_t index) const {
    return map[index];
  }

  size_t size() const {
    return static_cast<size_t>(n);
  }

  template <typename T>
  T* operator()(T *array) const {
    T temp[n];
    for(size_t i = 0; i < n; i++) temp[i] = array[i];
    for(size_t i = 0; i < n; i++) array[i] = temp[map[i]];
    return array;
  }

  template <typename T>
  T* invert(T *array) const {
    T temp[n];
    for(size_t i = 0; i < n; i++) temp[i] = array[i];
    for(size_t i = 0; i < n; i++) array[map[i]] = temp[i];
    return array;
  }

protected:
  typedef std::vector<size_t> Table;
  Table map;
};

typedef Mapping<3> Mapping3;
std::ostream& operator<<(std::ostream& os, const MappingBase& map);
std::istream& operator>>(std::istream& is, MappingBase& map);

} // namespace uxvcos

#endif // UXVCOS_MAPPING_H
