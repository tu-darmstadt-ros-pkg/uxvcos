#include "Timestamp.h"
#include <time.h>

namespace Data {

Timestamp::ticks Timestamp::reference = Timestamp::getTicks();

Timestamp::ticks Timestamp::getTicks() {
#ifdef OROCOS_TARGET
  return RTT::os::TimeService::Instance()->getTicks();
#else
#ifdef SYSTEM_UNIX
  struct timespec t;
  if (clock_gettime(CLOCK_REALTIME, &t) < 0) return 0;
  return (ticks) t.tv_sec * 1000000000ll + (ticks) t.tv_nsec;
#else
  //throw std::runtime_error("Data::Timestamp::getTicks() is not implemented for this OS!");
  return 0;
#endif
#endif
}

} // namespace Data
