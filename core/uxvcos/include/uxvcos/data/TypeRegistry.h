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

#include <uxvcos/uxvcos.h>

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
