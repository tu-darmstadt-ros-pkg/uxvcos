#ifndef STREAM_BUFFER_H
#define STREAM_BUFFER_H

#ifdef OROCOS_TARGET
  #include <rtt/os/Mutex.hpp>
  #include <rtt/os/MutexLock.hpp>
#endif

#include <uxvcos/stream/Chunk.h>
#include <uxvcos/stream/Stream.h>
#include <string.h>

#include <uxvcos/uxvcos.h>

#ifdef EOF
  #undef EOF
#endif

template<typename T>
class UXVCOS_API IBuffer : public Stream
{
public:
  typedef T element_t;
  typedef unsigned int size_t;
  typedef int ssize_t;
  typedef unsigned int index_t;

  IBuffer() {}
  virtual ~IBuffer() {}

  // static const element_t EOF = 0;

  virtual element_t get() = 0;
  virtual InStream& get(element_t& c) = 0;
  //virtual bool find(element_t delim) = 0;
  //virtual bool find(element_t delim[], size_t n) = 0;
  virtual InStream& read(element_t* c, size_t n, element_t delim) = 0;
  virtual InStream& read(void *destination, size_t n) = 0;
  virtual size_t readsome(void *destination, size_t n) = 0;

  virtual OutStream& put(element_t c) = 0;
  virtual OutStream& push_front(element_t c) = 0;
  virtual OutStream& write(const void *source, size_t n) = 0;

  virtual size_t size() const = 0;
  virtual size_t free() const = 0;
  virtual size_t capacity() const = 0;

  virtual bool found()    const = 0;
  virtual bool full()     const = 0;

  // Low-level access
  virtual element_t *add(ssize_t size) = 0;
  virtual element_t *del(ssize_t size) = 0;
  virtual element_t *data() = 0;
  virtual element_t *tail() = 0;
  virtual size_t tailfree() const = 0;

  virtual void insertChunk(const Chunk& chunk) = 0;
  virtual const Chunk getChunk() const = 0;

  virtual Stream& start() = 0;
  virtual Stream& finish() = 0;
  virtual void clear() = 0;
  virtual void reset() = 0;

  virtual void lock() = 0;
  virtual void unlock() = 0;
};

template<typename T = unsigned char>
class UXVCOS_API Buffer : public IBuffer<T>
{
public:
  typedef T element_t;
  typedef unsigned int size_t;
  typedef int ssize_t;
  typedef unsigned int index_t;

  Buffer(size_t maxSize = 1024) : maxSize(maxSize), ptr(0), end(0) {
    _data = new element_t[maxSize];
    deleteFlag = true;
  }

  Buffer(size_t maxSize, element_t *data, size_t size) : maxSize(maxSize), ptr(0), end(0)
  {
    _data = new element_t[maxSize];
    for(end = 0; end < size && end < maxSize; end++) _data[end] = data[end];
    deleteFlag = true;
  }

  Buffer(element_t *data, size_t size) : _data(data), maxSize(size), ptr(0), end(0)
  {
    deleteFlag = false;
  }

  virtual ~Buffer() {
    if (deleteFlag) delete[] _data;
  }

  virtual size_t size() const { return (end - ptr); }
  virtual size_t free() const { return maxSize - (end - ptr); }
  virtual size_t capacity() const { return maxSize; }

  virtual bool found() const { return _found; }
  virtual bool full() const { return end == maxSize; }

  virtual element_t get()
  {
    // MutexLock lock(this);
    this->_eof = false;
    this->_overflow = false;
    if (ptr == end)
    {
      this->_eof = true;
      return this->EoF;
    }
    return _data[ptr++];
  }

  virtual InStream& get(element_t& c)
  {
    c = get();
    return *this;
  }

  virtual InStream& read(element_t* c, size_t n, element_t delim)
  {
    // MutexLock lock(this);
    index_t i = 0;
    element_t res;

    _found = false;

    do {
      res = get();
      c[i++] = res;
    } while (!this->eof() && c[i-1] != delim && i != n);

    if (c[i-1] == delim) _found = true;
    return *this;
  }

  virtual InStream& read(void *destination, size_t n)
  {
    // MutexLock lock(this);
    if (size() < n) {
      this->_eof = true;
      return *this;
    }

    this->_eof = false;
    this->_overflow = false;

    if (destination) memcpy(destination, _data + ptr, n * sizeof(element_t));
    ptr += n;
    return *this;
  }

  virtual size_t readsome(void *destination, size_t n)
  {
    // MutexLock lock(this);
    size_t len = (n <= size() ? n : size());
    read(destination, len);
    return len;
  }

  virtual element_t operator[](index_t i)
  {
    // MutexLock lock(this);
    if (i < size()) return _data[ptr + i];
    return this->EoF;
  }

  virtual OutStream& push_front(element_t c)
  {
    // MutexLock lock(this);

    if (ptr > 0) {
      _data[--ptr] = c;
    } else {
      this->_overflow = true;
    }
    return *this;
  }

  virtual OutStream& put(element_t c)
  {
    // MutexLock lock(this);
    cleanup(1);

    this->_eof = false;
    this->_overflow = false;

    if (end == maxSize) {
      this->_overflow = true;
    } else {
      _data[end++] = c;
    }

    return *this;
  }

  virtual OutStream& write(const void *source, size_t n) {
    // MutexLock lock(this);
    cleanup(n);

    this->_eof = false;
    this->_overflow = false;

    if (end + n > maxSize) {
      this->_overflow = true;
    } else {
      memcpy(_data + end, source, n * sizeof(element_t));
      end += n;
    }
    return *this;
  }

  virtual element_t *add(ssize_t size) {
    // MutexLock lock(this);
    if (size < 0) return 0;

    cleanup(size);
    element_t *tail = this->tail();
    if (end + size > maxSize) {
      this->_overflow = true;
      return 0;
    }
    
    this->_overflow = false;
    end += size;

    return tail;
  }

  virtual element_t *del(ssize_t size) {
    // MutexLock lock(this);
    if (size < 0) return 0;

    element_t *data = this->data();
    if (ptr + size > end) {
      this->_eof = true;
      return 0;
    }
    
    this->_eof = false;
    ptr += size;

    return data;
  }

  virtual element_t *data() {
    return _data + ptr;
  }

  virtual const element_t *data() const {
    return _data + ptr;
  }

  virtual element_t *tail() {
    return _data + end;
  }

  virtual const element_t *tail() const {
    return _data + end;
  }

  virtual size_t tailfree() const { return maxSize - end; }

  virtual void insertChunk(const Chunk& chunk) {
    write(chunk.data(), chunk.size());
  }

  virtual const Chunk getChunk() const {
    return Chunk(data(), size());
  }

  virtual Stream& start() {
    lock();
    return *this;
  }

  virtual Stream& finish() {
    unlock();
    return *this;
  }

  virtual void clear()
  {
    // MutexLock lock(this);
    ptr = 0;
    end = 0;
    this->_eof = false;
    this->_overflow = false;
  }

  virtual void reset() {
    clear();
    this->error(false);
  }

#ifdef OROCOS_TARGET
  virtual void lock() { mutex.lock(); }
  virtual void unlock() { mutex.unlock(); }

  class MutexLock : public RTT::os::MutexLock {
    public:
      MutexLock(Buffer<T> *buffer) : RTT::os::MutexLock(buffer->mutex) {}
  };

#else
  virtual void lock() { }
  virtual void unlock() { }

  class MutexLock {
    public:
      MutexLock(Buffer<T> *buffer) {}
  };
#endif

  void cleanup(size_t free = 0)
  {
    if (ptr > 0 && ptr == end)
    {
      clear();
    }

    if (ptr > 0 && (free == 0 || end + free > maxSize))
    {
      memcpy(_data, _data + ptr, (end - ptr) * sizeof(element_t));
      end -= ptr;
      ptr = 0;
      this->_eof = false;
      this->_overflow = false;
    }
  }

protected:
  const size_t maxSize;
  element_t* _data;
  index_t ptr;
  index_t end;

  bool deleteFlag;

  bool _found;

#ifdef OROCOS_TARGET
  RTT::os::MutexRecursive mutex;
#endif
};

template<typename T = unsigned char>
class StaticBuffer : public Buffer<T> {
public:
  StaticBuffer() : Buffer<T>(buffer, 1024) {}
  virtual ~StaticBuffer() {}

private:
  T buffer[1024];
};

template<typename T>
static inline InStream& operator>>(InStream& in, Buffer<T>& buffer) {
  typename Buffer<T>::MutexLock lock(&buffer);
  buffer.cleanup();
  size_t n = in.readsome(buffer.tail(), buffer.tailfree());
  buffer.add(n);
  return in;
}

template<typename T>
static inline OutStream& operator<<(OutStream& out, Buffer<T>& buffer) {
  typename Buffer<T>::MutexLock lock(&buffer);
  if (out.write(buffer.data(), buffer.size())) {
    buffer.clear();
  }
  return out;
}

template<typename T>
static inline OutStream& operator<<(Buffer<T>& buffer, InStream& in) {
  in >> buffer;
  return buffer;
}

template<typename T>
static inline InStream& operator>>(Buffer<T>& buffer, OutStream& out) {
  out << buffer;
  return buffer;
}

#endif // STREAM_BUFFER_H
