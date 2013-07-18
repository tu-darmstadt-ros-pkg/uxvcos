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

#ifndef STREAM_PROPERTYSTREAM_H
#define STREAM_PROPERTYSTREAM_H

#include <uxvcos/stream/Stream.h>

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Logger.hpp>
#include <uxvcos/system/systemcalls.h>

#include <uxvcos/uxvcos.h>

class UXVCOS_API PropertyInStream : public InStream {
  public:
    PropertyInStream(const RTT::PropertyBag* bag) : bag(bag), fieldNames(0), field(0) {}
    ~PropertyInStream() {}

    PropertyInStream& compose(Data::Streamable& data);
    
    template <typename T>
    InStream& composeField(T &value) {
      RTT::Property<T>* property = 0;
      
      if (fieldNames != 0 && fieldNames[field] != 0) {
        std::string fieldname(fieldNames[field]);

        property = bag->getPropertyType<T>(fieldname);
        if (property) {
          value = property->get();
          ++field;
          return *this;
        } else {
          RTT::log(RTT::Debug) << "  ... not found!" << RTT::endlog();
        }
      }
      
      // _error = -1;
      ++field;
      return *this;
    }
    
    virtual InStream& operator>>(float &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(double &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(bool &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(signed char &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(unsigned char &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(signed short &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(unsigned short &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(signed int &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(size_t &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(signed long &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(unsigned long &value) {
      return composeField(value);
    }

  private:
    const RTT::PropertyBag* bag;
    const char* const* fieldNames;
    int field;
};

class UXVCOS_API PropertyOutStream : public OutStream {
  public:
    PropertyOutStream(RTT::PropertyBag* bag) : bag(bag), fieldNames(0), field(0) {}
    ~PropertyOutStream() {}
    
    PropertyOutStream& decompose(const Data::Streamable& data);
    
    template <typename T>
    OutStream& decomposeField(const T &value) {
      std::string fieldname;

      if (fieldNames != 0 && fieldNames[field] != 0) {
        fieldname = fieldNames[field];
      } else {
        char name[10];
        snprintf(name, sizeof(name) - 1, "Field%d", field);
        fieldname = name;
      }
      
      bag->add(new RTT::Property<T>(fieldname, "", value));

      ++field;
      return *this;
    }

    virtual OutStream& operator<<(float value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(double value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(bool value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(signed char value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(unsigned char value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(signed short value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(unsigned short value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(signed int value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(size_t value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(signed long value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(unsigned long value) {
      return decomposeField(value);
    }

    virtual OutStream& operator<<(const std::string& s) {
      return decomposeField(s);
    }

  private:
    RTT::PropertyBag* bag;
    const char* const* fieldNames;
    int field;
};

PropertyInStream& operator>>(PropertyInStream& in, Data::Streamable& data);
PropertyOutStream& operator<<(PropertyOutStream& out, const Data::Streamable& data);

#endif // STREAM_PROPERTYSTREAM_H
