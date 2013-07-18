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

#ifndef INTERFACE_ETHERNETSENSORBASE_H
#define INTERFACE_ETHERNETSENSORBASE_H

#include <iostream>
#include <stdint.h>
#include <map>
#include <set>

#include <uxvcos/interface/RateControl.h>
#include "arm-interface.h"

namespace uxvcos {
namespace Interface {

typedef unsigned MessageId;

class EthernetInterface;
class EthernetSensorBase;
class EthernetSensorContainer;
//typedef boost::shared_ptr<EthernetSensorBase> EthernetSensorPtr;
typedef EthernetSensorBase *EthernetSensorPtr;
typedef request_t Request[2];

class EthernetSensorBase
{
public:
  EthernetSensorBase(EthernetInterface *interface);
  virtual ~EthernetSensorBase();

  virtual void addTo(EthernetSensorContainer& sensors);
  virtual EthernetSensorPtr sensor() { return this; }
  // virtual EthernetSensorPtr sensor() = 0;

  virtual bool decode(void *payload, size_t length, MessageId id) = 0;

  virtual void enable() { enabled = true; }
  virtual void disable() { enabled = false; }

  virtual bool isEnabled() const { return enabled; }
  virtual bool isTime() const { return isEnabled() && rate.isTime(); }

  virtual const std::string &getName() const = 0;
  virtual MessageId getMessageId() const = 0;
  virtual void request(Request &request) const = 0;

  virtual std::ostream& operator>>(std::ostream& os) const = 0;
  virtual RTT::Logger& log() const = 0;

protected:
  virtual void construct();

  EthernetInterface* interface;
  RateControl rate;

private:
  bool enabled;
};

class EthernetSensorContainer
{
public:
  typedef std::set<EthernetSensorPtr> Set;
  typedef std::multimap<MessageId,EthernetSensorPtr> Map;

  typedef Set::iterator set_iterator;
  set_iterator begin() { return set.begin(); }
  set_iterator end() { return set.end(); }

  typedef Map::iterator map_iterator;
  map_iterator lower_bound(const MessageId& id) { return map.lower_bound(id); }
  map_iterator upper_bound(const MessageId& id) { return map.upper_bound(id); }

  set_iterator insert(EthernetSensorPtr sensor);
  map_iterator insert(MessageId id, EthernetSensorPtr sensor);

private:
  Set set;
  Map map;
};

static inline std::ostream& operator<<(std::ostream& os, const EthernetSensorBase &sensor) {
  return sensor.operator >>(os);
}

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETSENSORBASE_H
