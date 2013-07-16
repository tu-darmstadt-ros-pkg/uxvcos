#include "EthernetSensor.h"
#include "EthernetInterface.h"

#include <set>

namespace uxvcos {
namespace Interface {

EthernetSensorBase::EthernetSensorBase(EthernetInterface *interface)
  : interface(interface)
  , rate(interface->getRateControl())
  , enabled(true)
{}

EthernetSensorBase::~EthernetSensorBase()
{}

void EthernetSensorBase::construct() {
  Module *module = dynamic_cast<Module *>(this);
  if (!module) return;

  module->properties()->addProperty(rate.rate);
  module->properties()->addProperty(rate.offset);
  module->addOperation("enable", &EthernetSensorBase::enable, this);
  module->addOperation("disable", &EthernetSensorBase::disable, this);
  module->addOperation("isEnabled", &EthernetSensorBase::isEnabled, this);
}

void EthernetSensorBase::addTo(EthernetSensorContainer& map) {
  map.insert(getMessageId(), sensor());
}

EthernetSensorContainer::set_iterator EthernetSensorContainer::insert(EthernetSensorPtr sensor) {
  return set.insert(sensor).first;
}

EthernetSensorContainer::map_iterator EthernetSensorContainer::insert(MessageId id, EthernetSensorPtr sensor) {
  set.insert(sensor);
  return map.insert(Map::value_type(id, sensor));
}

} // namespace Interface
} // namespace uxvcos
