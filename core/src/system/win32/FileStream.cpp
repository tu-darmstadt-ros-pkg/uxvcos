#include "FileStream.h"
#include <limits.h>

namespace System {
namespace Win32 {

InFileStream::InFileStream()
{
}

InFileStream::InFileStream(const std::string filename)
  : filename(filename)
{
}

InFileStream::~InFileStream()
{
}

BaseStream::element_t InFileStream::get() {
  return static_cast<element_t>(in.get());
}

InStream& InFileStream::read(void *destination, BaseStream::size_t n) {
  in.read(static_cast<char *>(destination), n);
  return *this;
}

BaseStream::size_t InFileStream::readsome(void *destination, BaseStream::size_t n) {
  in.read(static_cast<char *>(destination), n);
  return in.gcount();
}

BaseStream::size_t InFileStream::size() const {
  return UINT_MAX;

  // if (!in.rdbuf()) return 0;
  // if (in.rdbuf()->in_avail() < 0) return 0;
  // return in.rdbuf()->in_avail();
}

bool InFileStream::open() {
  if (isOpen()) return true;
  in.clear();
  in.open(filename.c_str(), std::ios::in | std::ios::binary);

  return in.good();
}

bool InFileStream::open(const std::string filename) {
  close();
  this->filename = filename;
  return open();
}

void InFileStream::close() {
  in.close();
}

OutFileStream::OutFileStream()
{
}

OutFileStream::OutFileStream(const std::string filename)
  : filename(filename)
{
}

OutFileStream::~OutFileStream()
{
}

OutStream& OutFileStream::write(const void *source, BaseStream::size_t size) {
  out.write(static_cast<const char *>(source), size);
  return *this;
}

OutStream& OutFileStream::put(unsigned char c) {
  out.put(c);
  return *this;
}

bool OutFileStream::open() {
  if (isOpen()) return true;
  out.clear();
  out.open(filename.c_str(), std::ios::out | std::ios::binary);
  return out.good();
}

bool OutFileStream::open(const std::string filename) {
  close();
  this->filename = filename;
  return open();
}

void OutFileStream::close() {
  out.close();
}

} // namespace Win32
} // namespace System
