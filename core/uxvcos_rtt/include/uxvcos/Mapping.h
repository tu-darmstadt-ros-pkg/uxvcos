#ifndef UXVCOS_MAPPING_H
#define UXVCOS_MAPPING_H

#include <rtt/TaskContext.hpp>
#include <vector>

namespace uxvcos {

class MappingBase {
public:
  static const size_t VARIABLE = 0;

  virtual ~MappingBase() {}
  virtual size_t& operator[](size_t index) = 0;
  virtual const size_t& operator[](size_t index) const = 0;
  virtual size_t size() const = 0;
  virtual void set_size(size_t size) = 0;

  std::string toString() const;
  bool fromString(const std::string& s);
};

class Mapping : public MappingBase {
public:
  Mapping(size_t size = VARIABLE)
    : _size(size)
    , map(size)
  {
    for(size_t i = 0; i < map.size(); i++) map[i] = i;
  }
  virtual ~Mapping() {}

  Mapping& operator=(const Mapping &other) {
    this->_size = other._size;
    this->map = other.map;
    return *this;
  }

  size_t& operator[](size_t index) {
    if (_size == VARIABLE && index >= map.size()) map.resize(index+1);
    return map[index];
  }

  const size_t& operator[](size_t index) const {
    return map[index];
  }

  size_t size() const {
    return static_cast<size_t>(map.size());
  }

  void set_size(size_t size) {
    if (_size != VARIABLE) return;
    _size = size;
    map.resize(_size);
  }

  template <typename T>
  T* operator()(T *array) const {
    T temp[map.size()];
    for(size_t i = 0; i < map.size(); i++) temp[i] = array[i];
    for(size_t i = 0; i < map.size(); i++) array[i] = temp[map[i]];
    return array;
  }

  template <typename T>
  T* invert(T *array) const {
    T temp[map.size()];
    for(size_t i = 0; i < map.size(); i++) temp[i] = array[i];
    for(size_t i = 0; i < map.size(); i++) array[map[i]] = temp[i];
    return array;
  }

private:
  size_t _size;

  typedef std::vector<size_t> Table;
  Table map;
};

std::ostream& operator<<(std::ostream& os, const MappingBase& map);
std::istream& operator>>(std::istream& is, MappingBase& map);

} // namespace uxvcos

#endif // UXVCOS_MAPPING_H
