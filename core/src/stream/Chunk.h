#ifndef STREAM_CHUNK_H
#define STREAM_CHUNK_H

#include <stream/Stream.h>
#include <uxvcos.h>

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
