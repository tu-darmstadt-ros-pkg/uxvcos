#ifndef DATA_KEY_H
#define DATA_KEY_H

#include <iostream>
#include <uxvcos/uxvcos.h>

namespace Data {

struct UXVCOS_API Key {
  unsigned int key;

  Key() : key(0) {}
  Key(unsigned int key) : key(key) {}
  Key(unsigned char classId, unsigned char messageId) : key(classId << 8 | messageId) {}
  Key(const Key& other) : key(other.key) {}

  operator unsigned int() const { return key; }
  // operator bool() const { return (key != 0); }
  operator void *() const { return reinterpret_cast<void *>(key != 0); }

  Key& operator=(const Key& other)        { key = other.key; return *this; }
  bool operator==(const Key& other) const { return (key == other.key); }
  bool operator!=(const Key& other) const { return (key != other.key); }
  bool operator<(const Key& other) const  { return (key < other.key); }

  unsigned int& operator=(const unsigned int& x)   { return key = x; }
  bool operator==(const unsigned int& other) const { return (key == other); }
  bool operator!=(const unsigned int& other) const { return (key != other); }
  bool operator<(const unsigned int& other) const  { return (key < other); }

  unsigned char classId() const   { return (key >> 8) & 0xFF; }
  unsigned char messageId() const { return key & 0xFF; }

  static const Key Invalid;
  static const Key Timestamp;
};

static inline std::ostream& operator<<(std::ostream& os, const Key& key) {
  return os << key.key;
}

static inline std::istream& operator>>(std::istream& is, Key& key) {
  return is >> key.key;
}

} // namespace Data

#endif // DATA_KEY_H
