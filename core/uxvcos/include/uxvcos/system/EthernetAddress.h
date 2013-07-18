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
