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

#ifndef SYSTEM_NETWORKADDRESS_H
#define SYSTEM_NETWORKADDRESS_H

#include <string>
#include <stdint.h>
#include "systemcalls.h"

#if defined(SYSTEM_UNIX)
  #include <arpa/inet.h>
#endif

#if defined(SYSTEM_WIN32)
  #include <Winsock2.h>
#endif

#include <uxvcos/uxvcos.h>

#include <iostream>

namespace System {
  
struct UXVCOS_API NetworkAddress {
  NetworkAddress()
    : valid(false), ip(0), port(0)
  {}

  NetworkAddress(std::string ip, uint16_t port)
    : valid(false), ip(0), port(0)
  {
    set(ip, port);
  }

  NetworkAddress(uint32_t ip, uint16_t port)
    : valid(true), ip(htonl(ip)), port(htons(port))
  {
  }

  virtual ~NetworkAddress() {}

  static NetworkAddress any(uint16_t port) {
    return NetworkAddress(0x00000000, port);
  }

  static NetworkAddress broadcast(uint16_t port)   {
    return NetworkAddress(0xFFFFFFFF, port);
  }

  static NetworkAddress local(uint16_t port)   {
    return NetworkAddress(0x7F000001, port);
  }

  bool set(std::string ip, uint16_t port = 0);
  
  std::string toString() const {
    char str[22] = "";
    snprintf(str, sizeof(str), "%u.%u.%u.%u:%u", (ip) & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF, ntohs(port));
    return std::string(str);
  }
  
  operator void*() { return (void*) valid; }

  bool valid;
  uint32_t ip; // network byte order!
  uint16_t port; // network byte order!
};

static inline std::ostream& operator<<(std::ostream& os, const NetworkAddress& address) {
  os << address.toString();
  return os;
}

} // namespace System

#endif // SYSTEM_NETWORKADDRESS_H
