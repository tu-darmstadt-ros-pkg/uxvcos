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
