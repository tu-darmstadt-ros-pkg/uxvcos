#ifndef SYSTEM_XENOMAI_H
#define SYSTEM_XENOMAI_H

#include <native/types.h>
#include <native/timer.h>
#include <rtdk.h>

#define XENOMAI_DEFAULT_TIMEOUT_MS 1000
#define XENOMAI_DEFAULT_TIMEOUT System::Xenomai::milliseconds(XENOMAI_DEFAULT_TIMEOUT_MS)

// erzwinge Verwendung von Realtime-Ausgabe
//#define XENO_FORCE_RTDK
#ifdef XENO_FORCE_RTDK
  #define printf(...)    rt_printf(__VA_ARGS__)
  #define fprintf(...)   rt_fprintf(__VA_ARGS__)
  #define vprintf(...)   rt_vprintf(__VA_ARGS__)
  #define vfprintf(...)  rt_vfprintf(__VA_ARGS__)
#endif

namespace System {
namespace Xenomai {

static inline RTIME milliseconds(const long ms) {
  if (ms == -1) return TM_INFINITE;
  if (ms == 0) return TM_NONBLOCK;
  return ((RTIME) ms) * ((RTIME) 1000000);
}

static const RTIME INFINITE = TM_INFINITE;
static const RTIME NONBLOCK = TM_NONBLOCK;

} // namespace Xenomai
} // namespace System

#endif // SYSTEM_XENOMAI_H
