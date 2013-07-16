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

#include <uxvcos.h>

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
