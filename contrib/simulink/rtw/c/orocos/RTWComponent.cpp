
#include "RTWComponent.hpp"
#include <rtt/Component.hpp>

ORO_CREATE_COMPONENT(RTW::RTW_COMPONENT)

    const char *RT_MEMORY_ALLOCATION_ERROR = "RTWComponent: memory allocation error.";

namespace RTW
{
    RTT::TaskContext* currentTC;
}

