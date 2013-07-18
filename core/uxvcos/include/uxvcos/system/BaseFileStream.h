#ifndef SYSTEM_BASEFILESTREAM_H
#define SYSTEM_BASEFILESTREAM_H

#include <uxvcos/uxvcos.h>

namespace System {
class UXVCOS_API BaseFileStream {
public:
  virtual ~BaseFileStream() {};

  virtual bool open() = 0;
  virtual bool open(const std::string filename) = 0;
  virtual void close() = 0;

  virtual bool isOpen() const = 0;
};
} // namespace System

#endif // BASEFILESTREAM_H
