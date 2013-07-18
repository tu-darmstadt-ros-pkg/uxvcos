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

#ifndef STREAM_CHUNK_H
#define STREAM_CHUNK_H

#include <uxvcos/stream/Stream.h>
#include <uxvcos/uxvcos.h>

// template <typename T = unsigned char>
class UXVCOS_API Chunk {
  public:
    typedef unsigned char element_t;
    typedef unsigned int size_t;

    Chunk() {}
    virtual ~Chunk() {}

    Chunk(const element_t *data, size_t size) :
      _data(data),
      _size(size) {}

    OutStream& operator>>(OutStream& out) const {
      return out.write(_data, _size);
    }

    virtual size_t size() const {
      return _size;
    }

    virtual const element_t* data() const {
      return _data;
    }

  protected:
    const element_t *_data;
    size_t _size;
};

// template <typename T = unsigned char>
class UXVCOS_API WriteableChunk : public Chunk {
  public:
    typedef unsigned char element_t;
    typedef unsigned int size_t;

    WriteableChunk() :
      Chunk()
    {}

    WriteableChunk(element_t *data, size_t size) :
      Chunk(data, size),
      _data(data)
    {}

    InStream& operator<<(InStream& in) const {
      in.readsome(_data, _size);
      return in;
    }

    virtual element_t* data() {
      return _data;
    }

  protected:
    element_t *_data;
};

#endif // STREAM_CHUNK_H
