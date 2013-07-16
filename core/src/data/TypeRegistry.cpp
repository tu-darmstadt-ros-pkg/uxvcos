#ifdef OROCOS_TARGET
  #include <rtt/types/Types.hpp>
  #include <rtt/Logger.hpp>
#endif // OROCOS_TARGET

#include <sstream>
#include <boost/algorithm/string.hpp>

#include "TypeRegistry.h"

namespace Data {

TypeRegistry *TypeRegistry::theInstance = 0;

TypeRegistry* TypeRegistry::Instance() {
  if (theInstance == 0) theInstance = new TypeRegistry();
  return theInstance;
}

void TypeRegistry::addType(TypeInfo *typeInfo, bool allowDuplicateKeys)
{
  addType(typeInfo, typeInfo->getKey(), allowDuplicateKeys);
}

void TypeRegistry::addType(TypeInfo *typeInfo, const Key& key, bool allowDuplicateKeys)
{
  #ifdef OROCOS_TARGET
    RTT::Logger::In in("TypeRegistry");
  #endif

  // check for duplicate key
  const TypeInfo* other = 0;
  if (!allowDuplicateKeys && key && (other = getTypeInfo(key)) != 0) {

    // if other TypeInfo is the same, everything is fine
    if (other == typeInfo) return;

    std::ostringstream message;
    message << "Duplicate key " << key << " for different types " << typeInfo->getTypeName() << " and " << other->getTypeName();
    
    delete typeInfo;

    #ifdef OROCOS_TARGET
      RTT::log( RTT::Warning ) << message.str() << RTT::endlog();
      //throw duplicate_key_exception(message.str());
      return;
    #else
      throw duplicate_key_exception(message.str());
    #endif
  }
 
  #ifdef OROCOS_TARGET
    RTT::log( RTT::Debug ) << "Adding type '" << typeInfo->getTypeName() << "' (Key = " << key << ", TypeId = " << typeInfo->getTypeIdName() << ") to the type system" << RTT::endlog();
  #endif

  if (key && registryByKey.count(key) == 0) registryByKey[key] = typeInfo;
  if (getTypeInfoById(typeInfo->getTypeId()) == 0) registryByTypeId[typeInfo->getTypeId()] = typeInfo;

#ifdef OROCOS_TARGET
  if (!RTT::types::Types()->type(typeInfo->getTypeName())) RTT::types::Types()->addType(typeInfo);
#endif
}

void TypeRegistry::addType(TypeInfoGenerator* typeInfo, bool allowDuplicateKeys)
{
  addType(typeInfo, typeInfo->getTypeInfoObject()->getKey(), allowDuplicateKeys);
}

void TypeRegistry::addType(TypeInfoGenerator* typeInfo, const Key& key, bool allowDuplicateKeys)
{
#ifdef OROCOS_TARGET
  RTT::types::Types()->addType(typeInfo->RTTTypeInfoGenerator());
#endif

  addType(typeInfo->getTypeInfoObject(), key, allowDuplicateKeys);
}

TypeInfo* TypeRegistry::getTypeInfo(const Key key)
{
  return registryByKey.count(key) > 0 ? registryByKey[key] : 0;
}

TypeInfo* TypeRegistry::getTypeInfoById(const std::type_info *type)
{
  return registryByTypeId.count(type) > 0 ? registryByTypeId[type] : 0;
}

TypeInfo* TypeRegistry::getTypeInfo(const std::string &name)
{
  std::map<Key,TypeInfo*>::iterator iter;

  for(iter = registryByKey.begin(); iter != registryByKey.end(); iter++)
  {
    if (iter->second->getTypeName() == name) return iter->second;
  }

  return 0;
}

} // namespace Data
