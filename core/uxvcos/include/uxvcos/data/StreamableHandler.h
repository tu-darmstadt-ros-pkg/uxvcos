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

#ifndef DATA_STREAMABLEHANDLER_H
#define DATA_STREAMABLEHANDLER_H

#include "TypeInfo.h"
#include "TypeRegistry.h"

#ifdef OROCOS_TARGET
#include <rtt/internal/DataSources.hpp>
#endif // OROCOS_TARGET

namespace Data {

class StreamableHandler; // declaration in TypeInfo.h

template <typename T>
class UXVCOS_EXPORT TemplateStreamableHandler : public StreamableHandler
{
  virtual TypeInfo* getTypeInfo() const {
    return types()->getTypeInfo<T>();
  }

  virtual Streamable* create(void *place = 0) const {
    T* p;
    if (place)
      p = new(place) T();
    else
      p = new T();

    return dynamic_cast<Streamable *>(p);
  }

#ifdef OROCOS_TARGET
  typedef RTT::internal::DataSource<T>            DataSourceT;
  typedef RTT::internal::AssignableDataSource<T>  AssignableDataSourceT;

  virtual OutStream& serialize(OutStream& out, RTT::base::DataSourceBase::shared_ptr dsb) const {
    const T* datap = value(dsb);
    if (datap) {
      out << *datap;
    } else {
      T data;
      if (value(dsb, data))
        out << data;
      else
        out.error(true);
    }
    return out;
  }

  virtual InStream& deserialize(InStream& in, RTT::base::DataSourceBase::shared_ptr dsb) const {
    T* data = set(dsb);
    if (data) { in >> (*data); } else { in.error(true); }
    return in;
  }

  virtual RTT::base::DataSourceBase::shared_ptr buildValue() const {
    return new RTT::internal::ValueDataSource<T>();
  }

  virtual RTT::base::DataSourceBase::shared_ptr buildReference(void* ptr) const {
    return new RTT::internal::ReferenceDataSource<T>(*static_cast<T*>(ptr));
  }

  virtual const T* value(RTT::base::DataSourceBase::shared_ptr dsb) const {
    AssignableDataSourceT* ds = AssignableDataSourceT::narrow(dsb.get());
    if (!ds) return 0;
    return &(ds->rvalue());
  }

  virtual bool value(RTT::base::DataSourceBase::shared_ptr dsb, Streamable& value) const {
    DataSourceT* ds = DataSourceT::narrow(dsb.get());
    if (!ds) {
#ifndef NDEBUG
      RTT::log(RTT::Warning) << "Someone tried to get a Streamable of type " << getTypeInfo()->getTypeName() << " (" << getTypeInfo()->getTypeIdName() << ")" << " from a " << typeid(*(dsb.get())).name() << " DataSource of type " << dsb->getType() << " (" << dsb->getTypeInfo()->getTypeIdName() << ")" << RTT::endlog();
#endif
      return false;
    }
    dynamic_cast<T&>(value) = ds->value();
    return true;
  }

  virtual T* set(RTT::base::DataSourceBase::shared_ptr dsb) const {
    AssignableDataSourceT* ds = AssignableDataSourceT::narrow(dsb.get());
    if (!ds) return 0;
    return &(ds->set());
  }

  virtual bool set(RTT::base::DataSourceBase::shared_ptr dsb, const Streamable& data) const {
    AssignableDataSourceT* ds = AssignableDataSourceT::narrow(dsb.get());
    const T& source = dynamic_cast<const T&>(data);
    if (!ds) return false;
    ds->set(source);
    return true;
  }

#endif // OROCOS_TARGET
};

} // namespace Data

#endif // DATA_STREAMABLEHANDLER_H
