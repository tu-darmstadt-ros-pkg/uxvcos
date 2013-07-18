#ifndef STREAM_BASESTREAM_H
#define STREAM_BASESTREAM_H

#include <uxvcos/uxvcos.h>

class UXVCOS_API BaseStream
{
public:
  typedef unsigned char element_t;
  typedef unsigned int size_t;
  typedef int ssize_t;
  typedef unsigned int index_t;

  BaseStream() : _error(0) {}
  virtual ~BaseStream() {}

  virtual bool good() const {
    return _error == 0;
  }

  virtual operator void *() {
    return reinterpret_cast<void*>(good());
  }

  virtual bool open() {
    return true;
  }

  virtual void close() {
  }

  virtual void error(const int& i) {
    _error = i;
  }

  virtual void error(const bool& e) {
    _error = (!e ? 0 : (!_error ? -1 : _error));
  }

  virtual int error() const {
    return _error;
  }

  virtual void fail() {
    error(true);
  }

  virtual void reset() {
    error(false);
  }

  virtual const char *strerror() const {
    return strerror(error());
  }

  virtual const char *strerror(const int err) const {
    return 0;
  }

protected:
  int _error;
};

class OutStream;
class InStream;
class Stream;

#endif // STREAM_BASESTREAM_H
