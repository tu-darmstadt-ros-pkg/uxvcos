#ifndef UXVCOS_CONNECTOR_SOCKETCONNECTOR_H
#define UXVCOS_CONNECTOR_SOCKETCONNECTOR_H

#include <stream/Stream.h>
#include <stream/Buffer.h>
#include <stream/ubx/Stream.h>
#include <data/Streamable.h>

#include <system/NetworkAddress.h>
#include <system/Socket.h>
#include <system/Error.h>

#include <uxvcos.h>

#include "Connector.h"

namespace uxvcos {
class UXVCOS_API SocketConnector : public InConnector, public OutConnector {
public:
  enum Mode { Input, Output };

  SocketConnector();
  SocketConnector(BaseStream* stream);
  virtual ~SocketConnector();

  virtual bool listen(unsigned short port);
  virtual bool listen(const System::NetworkAddress& local);
  virtual bool connect(const System::NetworkAddress& remote);
  virtual bool broadcast(unsigned short port);

  virtual bool send(const Data::Streamable& message);
  virtual bool receive(Data::Streamable **streamable);

  virtual void close();

  virtual bool eof() const      { return !in || _eof; }
  virtual bool overflow() const { return !out || _overflow; }
  virtual bool timeout() const  { return _timeout; }

  virtual const System::Error& getError() const { return error; }

protected:
  InStream* in;
  OutStream* out;
  System::Socket* socket;

  UBX::Decoder* decoder;
  UBX::Encoder* encoder;
  Buffer<> inBuffer;
  Buffer<> *outBuffer;

  System::Error error;
  bool _timeout;
  bool _eof, _overflow;
};
} // namespace uxvcos

#endif // UXVCOS_CONNECTOR_SOCKETCONNECTOR_H

