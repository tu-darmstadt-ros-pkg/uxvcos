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

#ifndef STREAM_VECTORSTREAM_H
#define STREAM_VECTORSTREAM_H

#include <stream/Stream.h>
#include <vector>
#include <limits>

#include <uxvcos/uxvcos.h>

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
