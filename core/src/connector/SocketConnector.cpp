#include "SocketConnector.h"
#include <system/Error.h>

namespace uxvcos {

SocketConnector::SocketConnector()
  : in(0), out(0)
  , socket(0)
  , decoder(0), encoder(0)
  , inBuffer(65536), outBuffer(0)
  , error(0)
  , _timeout(false), _eof(true), _overflow(true)
{}

SocketConnector::SocketConnector(BaseStream* stream)
  : in(dynamic_cast<InStream*>(stream)), out(dynamic_cast<OutStream*>(stream))
  , socket(0)
  , decoder(0), encoder(0)
  , inBuffer(65536), outBuffer(0)
  , error(0)
  , _timeout(false), _eof(true), _overflow(true)
{
  if (in) {
    decoder = new UBX::Decoder(&inBuffer);
    _eof = false;
  }
  if (out) {
    encoder = new UBX::Encoder(out);
    _overflow = false;
  }
}

SocketConnector::~SocketConnector()
{
  close();
  delete outBuffer;
}

bool SocketConnector::listen(unsigned short port)
{
  return listen(System::NetworkAddress("0.0.0.0", port));
}

bool SocketConnector::listen(const System::NetworkAddress& local)
{
  if (socket != 0) return false;
  error = 0;

  socket = new System::Socket(System::Socket::SOCKET_UDP);
  if (!socket->create()) {
    error = System::Error::lastError();
    close();
    return false;
  }

  if (!socket->bind(local)) {
    error = System::Error::lastError();
    close();
    return false;
  }

  socket->setReuseAddr(true);
  socket->setNonBlocking(true);

  in = socket;
  decoder = new UBX::Decoder(&inBuffer);
  _eof = false;

  return true;
}

bool SocketConnector::connect(const System::NetworkAddress& remote)
{
  if (socket != 0) return false;
  error = 0;

  socket = new System::Socket(System::Socket::SOCKET_TCP);
  if (!socket->create()) {
    error = System::Error::lastError();
    close();
    return false;
  }

  if (!socket->connect(remote)) {
    error = System::Error::lastError();
    close();
    return false;
  }

  socket->setReuseAddr(true);
  socket->setBroadcast(true);
  socket->setNonBlocking(true);

  in = socket;
  decoder = new UBX::Decoder(&inBuffer);
  _eof = false;

  out = socket;
  if (!outBuffer) {
    encoder = new UBX::Encoder(out);
  } else {
    encoder = new UBX::Encoder(outBuffer);
  }
  _overflow = false;

  return true;
}

bool SocketConnector::broadcast(unsigned short port)
{
  if (socket != 0) return false;
  error = 0;

  socket = new System::Socket(System::Socket::SOCKET_UDP);
  if (!socket->create()) {
    error = System::Error::lastError();
    close();
    return false;
  }

  socket->setReuseAddr(true);
  socket->setBroadcast(true);
  socket->setNonBlocking(true);

  if (!socket->connect(System::NetworkAddress("255.255.255.255", port))) {
    error = System::Error::lastError();
    close();
    return false;
  }

  out = socket;
  if (!outBuffer) {
    encoder = new UBX::Encoder(out);
  } else {
    encoder = new UBX::Encoder(outBuffer);
  }
  _overflow = false;

  return true;
}

void SocketConnector::close()
{
  delete decoder;
  delete encoder;

  if (socket) {
    socket->close();
    delete socket;
    socket = 0;
  }

  in = 0;
  out = 0;

  inBuffer.clear();
  if (outBuffer) outBuffer->clear();
}

bool SocketConnector::receive(Data::Streamable **streamable) {
  if (!in || !decoder) return false;
  *streamable = 0;

  _timeout = false;
  _eof = false;

  while(1) {
    *streamable = decoder->read();
    if (*streamable == Data::Streamable::BREAK) continue;
    if (*streamable) return true;

    if (!in->wait(100)) {
      _timeout = true;
      return 0;
    }

    *in >> inBuffer;

    _eof = in->eof();
    if (_eof) return false;
  }

  return false;
}

bool SocketConnector::send(const Data::Streamable& message) {
  if (!out || !encoder) return false;
  bool result = encoder->write(message);
  _overflow = out->overflow();
  return result;
}

} // namespace uxvcos
