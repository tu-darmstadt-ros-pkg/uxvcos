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

#ifndef DATA_TYPEINFO_H
#define DATA_TYPEINFO_H

#include <uxvcos/uxvcos.h>
#include <typeinfo>

#ifdef OROCOS_TARGET
  #include <rtt/types/Types.hpp>
  #include <rtt/base/DataSourceBase.hpp>
#endif // OROCOS_TARGET

#include <uxvcos/stream/Stream.h>
#include <uxvcos/data/Key.h>

namespace Data {

class TypeInfo;
class TypeInfoGenerator;

class UXVCOS_API StreamableHandler {
public:
  virtual TypeInfo* getTypeInfo() const = 0;

  virtual Streamable* create(void *place = 0) const = 0;
  virtual OutStream& serialize(OutStream& out, const Streamable* data);
  virtual InStream& deserialize(InStream& in, Streamable* data);

#ifdef OROCOS_TARGET
  virtual OutStream& serialize(OutStream& out, RTT::base::DataSourceBase::shared_ptr dsb) const = 0;
  virtual InStream& deserialize(InStream& in, RTT::base::DataSourceBase::shared_ptr dsb) const = 0;

  virtual const Streamable* value(RTT::base::DataSourceBase::shared_ptr dsb) const = 0;
  virtual bool value(RTT::base::DataSourceBase::shared_ptr dsb, Streamable& value) const = 0;
  virtual bool set(RTT::base::DataSourceBase::shared_ptr dsb, const Streamable& data) const = 0;
  virtual Streamable* set(RTT::base::DataSourceBase::shared_ptr dsb) const = 0;
#endif // OROCOS_TARGET
};

#ifdef OROCOS_TARGET
class UXVCOS_API TypeInfo : public RTT::types::TypeInfo
{
private:
  std::string _short;
  std::string _description;
  Key _key;

public:
  TypeInfo(const std::string& name) : RTT::types::TypeInfo(check_name(name)) {}

#else
class UXVCOS_API TypeInfo
{
public:
  typedef const std::type_info * TypeId;

private:
  std::string _name;
  std::string _short;
  std::string _description;
  Key _key;
  const char* _tid_name;
  TypeId _tid;

public:
  TypeInfo(const std::string& name) : _name(check_name(name)), _tid_name(0), _tid(0) {}

  const std::string& getTypeName() const { return _name; }
  TypeId getTypeId() const { return _tid; }
  const char *getTypeIdName() const { return _tid_name; }

  void setTypeId(TypeId tid) {
    _tid = tid;
    _tid_name = tid->name();
  }

  TypeInfo& setName(const std::string& name) {
    _name = name;
    return *this;
  }

#endif

  std::string getShortName() const { return !_short.empty() ? _short : getTypeName(); }
  std::string getDescription() const { return _description; }
  Key getKey(std::string tag = std::string()) const { return _key; }

  TypeInfo& setShortName(const std::string& shortName) { _short = shortName; return *this; }
  TypeInfo& setKey(Key key) { _key = key; return *this; }
  TypeInfo& setDescription(const std::string& description) { _description = description; return *this; }

  unsigned char getClassId() const {
    return getKey().classId();
  }

  unsigned char getMessageId() const {
    return getKey().messageId();
  }

  Streamable* create(void *place = 0) const {
    return _streamable_handler ? _streamable_handler->create(place) : 0;
  }

  OutStream& serialize(OutStream& out, const Streamable* data) {
    return _streamable_handler ? _streamable_handler->serialize(out, data) : out;
  }

  InStream& deserialize(InStream& in, Streamable* data) {
    return _streamable_handler ? _streamable_handler->deserialize(in, data) : in;
  }

#ifdef OROCOS_TARGET
  OutStream& serialize(OutStream& out, RTT::base::DataSourceBase::shared_ptr dsb) const {
    return _streamable_handler ? _streamable_handler->serialize(out, dsb) : out;
  }

  InStream& deserialize(InStream& in, RTT::base::DataSourceBase::shared_ptr dsb) const {
    return _streamable_handler ? _streamable_handler->deserialize(in, dsb) : in;
  }

  const Streamable* value(RTT::base::DataSourceBase::shared_ptr dsb) const {
    return _streamable_handler ? _streamable_handler->value(dsb) : 0;
  }

  bool value(RTT::base::DataSourceBase::shared_ptr dsb, Streamable& value) const {
    return _streamable_handler ? _streamable_handler->value(dsb, value) : false;
  }

  bool set(RTT::base::DataSourceBase::shared_ptr dsb, const Streamable& data) const {
    return _streamable_handler ? _streamable_handler->set(dsb, data) : false;
  }

  Streamable* set(RTT::base::DataSourceBase::shared_ptr dsb) const {
    return _streamable_handler ? _streamable_handler->set(dsb) : 0;
  }

  RTT::types::TypeInfo* RTTTypeInfo() { return this; }
  static const TypeInfo *fromRTTTypeInfo(const RTT::types::TypeInfo* typeinfo);
#endif // OROCOS_TARGET

  TypeInfo& setStreamableHandler(const boost::shared_ptr<StreamableHandler>& handler) {
    _streamable_handler = handler;
    return *this;
  }

public:
  static std::string check_name(const std::string& name);

private:
  boost::shared_ptr<StreamableHandler> _streamable_handler;
};

} // namespace Data

#endif // DATA_TYPEINFO_H
