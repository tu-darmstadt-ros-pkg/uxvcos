#ifndef UXVCOS_CONNECTOR_CONNECTOR_H
#define UXVCOS_CONNECTOR_CONNECTOR_H

#include <data/Streamable.h>
#include <uxvcos.h>

namespace uxvcos {

class UXVCOS_API OutConnector {
public:
  OutConnector() {}
  virtual ~OutConnector() {}

  virtual bool send(const Data::Streamable& message) = 0;
};

class UXVCOS_API InConnector {
public:
  InConnector() {}
  virtual ~InConnector() {}

  virtual bool receive(Data::Streamable **streamable) = 0;

  virtual Data::Streamable *receive() {
    Data::Streamable *streamable;
    if (!receive(&streamable)) return 0;
    return streamable;
  }

};

} // namespace uxvcos

#endif // UXVCOS_CONNECTOR_CONNECTOR_H
