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
