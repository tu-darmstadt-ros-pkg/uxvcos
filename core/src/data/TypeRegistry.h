//!  Class Data::TypeRegistry
/*!
  This class implements methods to register and find data classes by id or name.
*/

#ifndef DATA_TYPEREGISTRY_H
#define DATA_TYPEREGISTRY_H

#include <map>
#include <string>
#include <stdexcept>

#include "TypeInfo.h"
#include "TypeInfoGenerator.h"

#include <uxvcos.h>

namespace Data {

class UXVCOS_API TypeRegistry {
  public:
    struct duplicate_key_exception : public std::runtime_error
    {
      duplicate_key_exception(const std::string &message) :
        std::runtime_error(message) {}
    };

    static TypeRegistry* Instance();

    // implemented in StreamableTypeInfo.h
    template <typename T> TypeInfo& addType(const std::string& name, const Key& key, const std::string& description = "", bool allowDuplicateKeys = false);
    void addType(TypeInfo* typeInfo, const Key& key, bool allowDuplicateKeys = false);
    void addType(TypeInfo* typeInfo, bool allowDuplicateKeys = false);
    void addType(TypeInfoGenerator* typeInfo, const Key& key, bool allowDuplicateKeys = false);
    void addType(TypeInfoGenerator* typeInfo, bool allowDuplicates = false);

    TypeInfo* getTypeInfo(const Key key);
    TypeInfo* getTypeInfo(const std::string &name);
    TypeInfo* getTypeInfoById(const std::type_info *type);

    template <typename T> TypeInfo* getTypeInfo() {
      return getTypeInfoById(&typeid(T));
    }

    template <typename T> TypeInfo* getTypeInfo(const T& ref) {
      return getTypeInfoById(&typeid(T));
    }

  private:
    std::map<Key,TypeInfo*>         registryByKey;
    std::map<const std::type_info *,TypeInfo*> registryByTypeId;
    static TypeRegistry *theInstance;
    
  public:
    class Register {
    public:
      Register(void (*callback)(void)) { callback(); }
    };
};

static inline TypeRegistry* types() {
  return TypeRegistry::Instance();
}

} // namespace Data

#endif // DATA_TypeREGISTRY_H
