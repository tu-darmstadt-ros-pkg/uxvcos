#ifndef DATA_STREAMABLETYPEINFO_H
#define DATA_STREAMABLETYPEINFO_H

#include <uxvcos.h>

#include "TypeInfo.h"
#include "TypeInfoGenerator.h"
#include "TypeRegistry.h"

#include "StreamableHandler.h"
#include "StreamableCompositionFactory.h"

#ifdef OROCOS_TARGET
  #include <rtt/types/TemplateTypeInfo.hpp>
  #include <rtt/types/TemplateConstructor.hpp>
#endif // OROCOS_TARGET

#include <typeinfo>

namespace Data {

#ifdef OROCOS_TARGET
template <typename T>
class UXVCOS_EXPORT StreamableTypeInfo : public RTT::types::TemplateTypeInfo<T,true>, public TypeInfoGenerator
{
public:
  StreamableTypeInfo(const std::string& name, const Key key = 0, const std::string& description = "")
    : RTT::types::TemplateTypeInfo<T,true>(Data::TypeInfo::check_name(name))
    , _typeinfo(new Data::TypeInfo(RTT::types::TemplateTypeInfo<T,true>::getTypeName()))
#else // !defined(OROCOS_TARGET)
template <typename T>
class UXVCOS_EXPORT StreamableTypeInfo : public TypeInfoGenerator
{
public:
  StreamableTypeInfo(const std::string& name, const Key key = 0, const std::string& description = "")
    : _typeinfo(new Data::TypeInfo(name))
#endif // OROCOS_TARGET
  {
    _typeinfo->setTypeId(&typeid(T));
    _typeinfo->setKey(key);
    _typeinfo->setDescription(description);
  }

  ~StreamableTypeInfo() {
  }

  /**
   * Return the type name for which this generator
   * generates type info features. This name will be
   * aliased by the TypeInfo object.
   */
  virtual const std::string& getTypeName() const {
    return _typeinfo->getTypeName();
  }

  /**
   * Installs the type info object in the global data source type info handler
   * and adds any additional features to the type info object.
   * This method will be called by the TypeInfoRepository, in order to register this
   * type's factories into the TypeInfo object.
   * @param ti A valid TypeInfo object into which new features
   * may be installed
   * @return true if this object may be deleted, false if not.
   * @post When true is returned, this instance is still valid and
   * the caller (TypeInfoRepository) will delete it. When false is returned,
   * the validity is undefined and the instance will not be used
   * anymore by the caller.
   */
  virtual bool installTypeInfoObject(TypeInfo* ti) {
    if (!this->RTT::types::TemplateTypeInfo<T,true>::installTypeInfoObject(ti)) return false;
    ti->setStreamableHandler(boost::make_shared<TemplateStreamableHandler<T> >());
    ti->setCompositionFactory(boost::make_shared<StreamableCompositionFactory<T> >(ti->getTypeName()));
    return false;
  }

  /**
   * Returns the TypeInfo object of this type, or null
   * if none exists yet.
   * @return All generators should return here TypeInfoRepository::Instance()->getTypeInfo<T>();
   */
  virtual TypeInfo* getTypeInfoObject() const {
    return _typeinfo;
  }

private:
  TypeInfo *_typeinfo;
};

template <typename T>
Data::TypeInfo& TypeRegistry::addType(const std::string& name, const Key& key, const std::string& description, bool allowDuplicates) {
  TypeInfoGenerator *typeInfo = new StreamableTypeInfo<T>(name, key, description);
  addType(typeInfo, key, allowDuplicates);
  return *(typeInfo->getTypeInfoObject());
}

} // namespace Data

#endif // DATA_STREAMABLETYPEINFO_H
