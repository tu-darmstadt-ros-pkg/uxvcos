#ifndef TYPES_TYPEKIT_H
#define TYPES_TYPEKIT_H

#include <uxvcos.h>
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
    typekit->loadTypes();
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

#ifdef OROCOS_TARGET
  #define UXVCOS_TYPEKIT_PLUGIN(typekit) ORO_TYPEKIT_PLUGIN(Data::typekit::Typekit)
#else
  #define UXVCOS_TYPEKIT_PLUGIN(typekit)
#endif

#endif // TYPES_TYPEKIT_H
