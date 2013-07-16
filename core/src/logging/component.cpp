#include <rtt/Component.hpp>

#include "Logging.h"
#include "Replay.h"

namespace uxvcos {
namespace Logging {

  ORO_LIST_COMPONENT_TYPE( Logging )
  ORO_LIST_COMPONENT_TYPE( Replay )

} // namespace Logging
} // namespace uxvcos

ORO_CREATE_COMPONENT_LIBRARY()
