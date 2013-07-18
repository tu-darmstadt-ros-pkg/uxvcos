#include "Socket.h"

bool System::NetworkAddress::set(std::string ip, uint16_t port) {
  struct hostent* host = gethostbyname(ip.c_str());
  if (!host) { valid = false; return false; }
  valid = true;
  this->ip = *(reinterpret_cast<unsigned long *>(host->h_addr_list[0]));
  if (port != 0) this->port = htons(port);
  return true;
}
