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

#ifndef TYPES_TYPEKIT_H
#define TYPES_TYPEKIT_H

#include <uxvcos/uxvcos.h>
#include <iostream>

#ifdef OROCOS_TARGET
  #include <rtt/types/TypekitPlugin.hpp>
  #include <rtt/Port.hpp>
  #include <rtt/Property.hpp>
  #include <rtt/Attribute.hpp>
  #include <rtt/internal/DataSources.hpp>
  #define TYPEKIT_BASE_CLASS : public RTT::types::TypekitPlugin
#else
  #define TYPEKIT_BASE_CLASS
#endif

#include <string>

namespace Data {

class UXVCOS_API Typekit TYPEKIT_BASE_CLASS {
public:
  virtual bool loadTypes() = 0;
  virtual bool loadOperators() { return true; }
  virtual bool loadConstructors() { return true; }
  virtual bool loadGlobals() { return true; }
  virtual std::string getName() = 0;
};

class UXVCOS_API TypekitLoader {
public:
  TypekitLoader(Typekit *typekit) {
    load(typekit);
  }
  TypekitLoader(const Typekit& typekit) {
    load(const_cast<Typekit *>(&typekit));
  }

  void load(Typekit *typekit) {
#ifndef OROCOS_TARGET
    typekit->loadTypes();
#else
    RTT::types::TypekitRepository::Import(typekit);
#endif
  }
};
}

#define UXVCOS_LOAD_TYPEKIT(typekit) do { static const Data::TypekitLoader temp(new Data::typekit::Typekit()); } while(0)

#define UXVCOS_DECLARE_TYPEKIT(typekit) \
  namespace Data { namespace typekit { \
    class UXVCOS_EXPORT Typekit : public Data::Typekit { \
    public: \
      bool loadTypes(); \
      std::string getName(); \
    }; \
  }}

#ifdef OROCOS_TARGET
  #ifndef UXVCOS_TYPEKIT
    #define UXVCOS_DECLARE_TYPE(type) \
      extern template class RTT::internal::DataSourceTypeInfo< type >; \
      extern template class RTT::internal::DataSource< type >; \
      extern template class RTT::internal::AssignableDataSource< type >; \
      extern template class RTT::internal::ValueDataSource< type >; \
      extern template class RTT::internal::ConstantDataSource< type >; \
      extern template class RTT::internal::ReferenceDataSource< type >; \
      extern template class RTT::OutputPort< type >; \
      extern template class RTT::InputPort< type >; \
      extern template class RTT::Property< type >; \
      extern template class RTT::Attribute< type >; \
      extern template class RTT::Constant< type >;
  #else
    #define UXVCOS_DECLARE_TYPE(type) \
      template UXVCOS_EXPORT class RTT::internal::DataSourceTypeInfo< type >; \
      template UXVCOS_EXPORT class RTT::internal::DataSource< type >; \
      template UXVCOS_EXPORT class RTT::internal::AssignableDataSource< type >; \
      template UXVCOS_EXPORT class RTT::internal::ValueDataSource< type >; \
      template UXVCOS_EXPORT class RTT::internal::ConstantDataSource< type >; \
      template UXVCOS_EXPORT class RTT::internal::ReferenceDataSource< type >; \
      template UXVCOS_EXPORT class RTT::OutputPort< type >; \
      template UXVCOS_EXPORT class RTT::InputPort< type >; \
      template UXVCOS_EXPORT class RTT::Property< type >; \
      template UXVCOS_EXPORT class RTT::Attribute< type >; \
      template UXVCOS_EXPORT class RTT::Constant< type >;
  #endif
#else
  #define UXVCOS_DECLARE_TYPE(type)
#endif

#if defined(OROCOS_TARGET) && defined(OROCOS_TYPEKIT)
  #define UXVCOS_TYPEKIT_PLUGIN(typekit) ORO_TYPEKIT_PLUGIN(Data::typekit::Typekit)
#else
  #define UXVCOS_TYPEKIT_PLUGIN(typekit)
#endif

#endif // TYPES_TYPEKIT_H
