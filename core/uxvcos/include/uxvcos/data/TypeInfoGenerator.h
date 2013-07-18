//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#ifndef DATA_TYPEINFOGENERATOR_H
#define DATA_TYPEINFOGENERATOR_H

#include <uxvcos/uxvcos.h>
#include "TypeInfo.h"

namespace Data {

class UXVCOS_API TypeInfoGenerator
{
public:
  /**
   * Return the type name for which this generator
   * generates type info features. This name will be
   * aliased by the TypeInfo object.
   */
  virtual const std::string& getTypeName() const = 0;

  /**
   * Returns the TypeInfo object of this type, or null
   * if none exists yet.
   * @return All generators should return here TypeInfoRepository::Instance()->getTypeInfo<T>();
   */
  virtual TypeInfo* getTypeInfoObject() const = 0;

#ifdef OROCOS_TARGET
  virtual RTT::types::TypeInfoGenerator* RTTTypeInfoGenerator() {
    return dynamic_cast<RTT::types::TypeInfoGenerator*>(this);
  }
#endif
};

} // namespace Data

#endif // DATA_TYPEINFOGENERATOR_H
