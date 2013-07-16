#ifndef STREAM_VECTORSTREAM_H
#define STREAM_VECTORSTREAM_H

#include <stream/Stream.h>
#include <vector>
#include <limits>

#include <uxvcos.h>

namespace Data { class UXVCOS_API Streamable; };

class UXVCOS_API VectorInStream : public InStream {
  public:
    VectorInStream(const std::vector<double> &vector) : vector(vector), field(0) { reset(); }
    ~VectorInStream() {}

    void reset() {
      InStream::reset();
      field = 0;
    }

    template <typename T>
    InStream& composeField(T &value) {
      if (field < vector.size()) {
        value = static_cast<T>(vector[field++]);
      } else {
        value = std::numeric_limits<T>::signaling_NaN();
        this->_eof = true;
      }
      return *this;
    }

    virtual InStream& operator>>(float &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(double &value) {
      return composeField(value);
    }

    virtual InStream& operator>>(bool &value) {
      double temp = 0.0;
      composeField(temp);
      value = (temp != 0.0);
      return *this;
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
    const std::vector<double> &vector;
    size_t field;
};

class UXVCOS_API VectorOutStream : public OutStream {
  public:
    VectorOutStream(std::vector<double> &vector) : vector(vector) { reset(); }
    ~VectorOutStream() {}

    void reset() {
      OutStream::reset();
      vector.clear();
    }

    template <typename T>
    OutStream& decomposeField(const T &value) {
      vector.push_back(static_cast<double>(value));
      this->_overflow = false;
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
      return *this;
    }

  private:
    std::vector<double> &vector;
};

UXVCOS_API VectorInStream& operator>>(VectorInStream& in, Data::Streamable& data);
UXVCOS_API VectorOutStream& operator<<(VectorOutStream& out, const Data::Streamable& data);

#endif // STREAM_VECTORSTREAM_H
