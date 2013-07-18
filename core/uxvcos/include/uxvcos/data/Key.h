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
