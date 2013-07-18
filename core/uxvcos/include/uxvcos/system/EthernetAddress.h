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

#ifndef SYSTEM_ETHERNETADDRESS_H
#define SYSTEM_ETHERNETADDRESS_H

#include <string>
#include <stdint.h>
#include "systemcalls.h"
#include <uxvcos/uxvcos.h>

namespace System {
struct UXVCOS_API EthernetAddress {
  EthernetAddress(std::string mac = "", uint16_t protocol = 0)
    : valid(false), protocol(0), ifindex(0)
  {
    memset(&addr, 0, sizeof(addr));
    set(mac, protocol);
  }

  EthernetAddress(uint8_t mac[6], uint16_t protocol = 0)
    : valid(true), protocol(htons(protocol)), ifindex(0)
  {
    memcpy(&addr, &mac, sizeof(addr));
  }

  virtual ~EthernetAddress() {}

  void set(std::string mac, uint16_t protocol = 0) {
    short unsigned int addr_short[6];
    if (sscanf(mac.c_str(), "%2hx:%2hx:%2hx:%2hx:%2hx:%2hx", &addr_short[0], &addr_short[1], &addr_short[2], &addr_short[3], &addr_short[4], &addr_short[5]) != 6 &&
        sscanf(mac.c_str(), "%2hx-%2hx-%2hx-%2hx-%2hx-%2hx", &addr_short[0], &addr_short[1], &addr_short[2], &addr_short[3], &addr_short[4], &addr_short[5]) != 6) {
      valid = false;
    } else {
      this->addr[0] = static_cast<uint8_t>(addr_short[0]);
      this->addr[1] = static_cast<uint8_t>(addr_short[1]);
      this->addr[2] = static_cast<uint8_t>(addr_short[2]);
      this->addr[3] = static_cast<uint8_t>(addr_short[3]);
      this->addr[4] = static_cast<uint8_t>(addr_short[4]);
      this->addr[5] = static_cast<uint8_t>(addr_short[5]);
      valid = true;
    }
    if (protocol != 0) this->protocol = htons(protocol);
  }

  void setInterface(const EthernetAddress& other) { this->ifindex = other.ifindex; }
  void setInterface(int ifindex) { this->ifindex = ifindex; }
  int getInterface() { return ifindex; }

  std::string toString() {
    char mac_string[23];
    if (!valid) return "<invalid>";
    snprintf(mac_string, sizeof(mac_string), "%02x:%02x:%02x:%02x:%02x:%02x:%04x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], ntohs(protocol));
    return std::string(mac_string);
  }

  operator void*() { return (void*) valid; }

  bool valid;
  uint16_t protocol;  /* Physical layer protocol */
  int ifindex;        /* Interface number */
  uint8_t addr[6];    /* Physical layer address */
};
} // namespace System

#endif // SYSTEM_ETHERNETADDRESS_H
