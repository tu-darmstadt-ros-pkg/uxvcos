#ifndef SYSTEM_WIN32_FILESTREAM_H
#define SYSTEM_WIN32_FILESTREAM_H

#include <fstream>
#include <string>

#include <stream/Stream.h>
#include <system/BaseFileStream.h>
#include <uxvcos/uxvcos.h>

namespace System {
namespace Win32 {

class UXVCOS_API InFileStream : public InStream, public BaseFileStream {
public:
  InFileStream();
  InFileStream(const std::string filename);
  virtual ~InFileStream();

  virtual element_t get();
  virtual InStream& read(void *destination, size_t n);
  virtual size_t readsome(void *destination, size_t n);
  virtual size_t size() const;

  virtual bool open();
  virtual bool open(const std::string filename);
  virtual void close();

  virtual bool eof() const {
    return in.eof();
  }

  virtual bool isOpen() const {
    return in.is_open();
  }

  virtual bool good() const {
    return isOpen() && InStream::good();
  }

protected:
  std::ifstream in;
  std::string filename;
};

class UXVCOS_API OutFileStream : public OutStream, public BaseFileStream {
public:
  OutFileStream();
  OutFileStream(const std::string filename);
  virtual ~OutFileStream();

  virtual OutStream& write(const void *source, size_t size);
  virtual OutStream& put(unsigned char c);

  virtual bool open();
  virtual bool open(const std::string filename);
  virtual void close();

  virtual bool overflow() const {
    return out.fail();
  }

  virtual bool isOpen() const {
    return out.is_open();
  }

  virtual bool good() const {
    return isOpen() && OutStream::good();
  }

protected:
  std::ofstream out;
  std::string filename;
};

} // namespace Win32
} // namespace System

#endif // SYSTEM_WIN32_FILESTREAM_H
