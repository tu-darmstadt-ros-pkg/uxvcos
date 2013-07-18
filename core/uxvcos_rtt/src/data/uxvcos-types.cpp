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

#include <uxvcos/data/TypeRegistry.h>
//#include <uxvcos/data/StreamableTypeInfo.h>
//#include <uxvcos/data/Timestamp.h>

#ifdef OROCOS_TARGET
  #include <rtt/types/TemplateTypeInfo.hpp>
  #include <rtt/types/SequenceTypeInfo.hpp>

  #include <uxvcos/MappingTypeInfo.h>
  #include <uxvcos/EchelonTypeInfo.h>
#endif // OROCOS_TARGET

// #define UXVCOS_TYPEKIT
#include "data/uxvcos-types.h"

namespace Data {
namespace uxvcos {
  bool Typekit::loadTypes() {
#ifdef OROCOS_TARGET
    // add some additional types that Orocos normally not knows
//    RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<Data::Timestamp,true>("Timestamp"));
    RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<signed short,true>("short"));
    RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<unsigned short,true>("ushort"));
    RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<signed long,true>("long"));
    RTT::types::Types()->addType(new RTT::types::TemplateTypeInfo<unsigned long,true>("ulong"));
    RTT::types::Types()->addType(new RTT::types::SequenceTypeInfo<std::vector<unsigned int>,false>("uints"));
    RTT::types::Types()->addType(new MappingTypeInfo());
    RTT::types::Types()->addType(new Echelon::TypeInfo());
#endif // OROCOS_TARGET

//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableVector<double> >("Vector"), true);
//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableVector<int> >("IntVector"), true);
//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableVector<unsigned short> >("UShortVector"), true);
//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableValue<double> >("Double"), true);
//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableValue<int> >("Int"), true);
//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableValue<unsigned short> >("UShort"), true);

//    Data::types()->addType(new Data::StreamableTypeInfo<Data::StreamableVector<double> >("Log", 0xFF));
    return true;
  }

  std::string Typekit::getName() {
    return "uxvcos-types";
  }

} // namespace uxvcos
} // namespace Data

// load typekit statically
#include <rtt/os/StartStopManager.hpp>
#include <rtt/types/TypekitRepository.hpp>
namespace {
  int loadTypekit() { RTT::types::TypekitRepository::Import(new Data::uxvcos::Typekit); return 0; }
  static RTT::os::InitFunction f(&loadTypekit);
}

UXVCOS_TYPEKIT_PLUGIN(uxvcos)
