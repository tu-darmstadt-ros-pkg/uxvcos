#include <uxvcos/stream/Stream.h>
#include <uxvcos/data/TypeInfo.h>

#include <rtt/Logger.hpp>

OutStream& operator<<(OutStream& out, RTT::base::DataSourceBase::shared_ptr dsb) {
  const Data::TypeInfo* typeInfo = Data::TypeInfo::fromRTTTypeInfo(dsb->getTypeInfo());
  if (!typeInfo) {
    // RTT::log(RTT::RealTime) << "Unknown type for outgoing stream: " << dsb->getType() << RTT::endlog();
    return out;
  }

  // RTT::log(RTT::RealTime) << "Trying to serialize object of type " << dsb->getType() << RTT::endlog();
  return typeInfo->serialize(out, dsb);
}

InStream& operator>>(InStream& in, RTT::base::DataSourceBase::shared_ptr dsb) {
  const Data::TypeInfo* typeInfo = Data::TypeInfo::fromRTTTypeInfo(dsb->getTypeInfo());
  if (!typeInfo) {
    // RTT::log(RTT::RealTime) << "Unknown type for incoming stream: " << dsb->getType() << RTT::endlog();
    return in;
  }

  // RTT::log(RTT::RealTime) << "Trying to deserialize object of type " << dsb->getType() << RTT::endlog();
  return typeInfo->deserialize(in, dsb);
}
