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

#ifndef INTERFACE_ETHERNETSENSOR_H
#define INTERFACE_ETHERNETSENSOR_H

#include "EthernetSensorBase.h"
#include <uxvcos/data/Key.h>
#include <uxvcos/Module.h>

namespace uxvcos {
namespace Interface {

template <typename SensorClass>
class EthernetSensor : public SensorClass, public EthernetSensorBase
{
public:
  EthernetSensor(Interface::EthernetInterface* owner, const std::string& name = "", const std::string& port_name = "", const std::string& description = "", MessageId messageId = 0, Data::Key key = Data::Key())
    : SensorClass(owner, name, port_name, description)
    , EthernetSensorBase(owner)
    , messageId(messageId), key(key)
  {
    construct();
  }

  EthernetSensor(Interface::EthernetInterface* owner, MessageId messageId = 0, Data::Key key = Data::Key())
    : SensorClass(owner)
    , EthernetSensorBase(owner)
    , messageId(messageId), key(key)
  {
    construct();
  }

  virtual ~EthernetSensor() {}

//  EthernetSensorPtr sensor() {
//    return boost::dynamic_pointer_cast<EthernetSensorBase>(this->shared_from_this());
//  }

  virtual bool initialize() { return SensorClass::initialize(); }
  virtual void cleanup() { SensorClass::cleanup(); }

  virtual bool decode(void *payload, size_t length, MessageId id);

  virtual const std::string &getName() const { return this->SensorClass::getName(); }
  virtual MessageId getMessageId() const { return messageId; }

  virtual Data::Key getKey() const { return key; }
  virtual void request(Request &request) const { SET_REQUEST(getMessageId(), request); }

  virtual std::ostream& operator>>(std::ostream& os) const { return os << *((SensorClass *)this); }
  virtual RTT::Logger& log() const { return SensorClass::log(); }

  virtual const typename SensorClass::ValueType& get() const { return SensorClass::get(); }

protected:
  const MessageId messageId;
  Data::Key key;
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETSENSOR_H
