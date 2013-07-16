#include "Priorities.h"

#ifdef OROCOS_TARGET
#include <rtt/os/threads.hpp>

namespace System {
namespace Priority {
  const int Lowest  = RTT::os::LowestPriority;
  const int Highest = RTT::os::HighestPriority;
  const int Normal  = (Lowest + Highest) / 2;
  const int Lower   = Normal - RTT::os::IncreasePriority;
  const int Higher  = Normal + RTT::os::IncreasePriority;
}
}

#endif