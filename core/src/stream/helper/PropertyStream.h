#ifndef STREAM_PROPERTYSTREAM_H
#define STREAM_PROPERTYSTREAM_H

#ifdef OROCOS_TARGET

#include <stream/Stream.h>

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>
#include <rtt/Logger.hpp>
#include <system/systemcalls.h>

#include <uxvcos.h>

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

#endif // OROCOS_TARGET
#endif // STREAM_PROPERTYSTREAM_H
