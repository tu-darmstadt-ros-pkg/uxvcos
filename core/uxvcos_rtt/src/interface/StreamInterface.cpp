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

#include "interface/StreamInterface.h"

#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/os/fosi.h>

#include <uxvcos/stream/ubx/Stream.h>
#include <uxvcos/data/TypeInfo.h>

#include <rtt/Component.hpp>

using namespace RTT;

namespace uxvcos {
namespace Interface {

StreamInterface::StreamInterface(const std::string &name, BaseStream *stream)
  : BaseInterface(name)
  , sync("Sync", "Sync characters in hexadecimal notation")
  , dataPool(DataPool::Global())
  , inStream(dynamic_cast<InStream *>(stream)), outStream(dynamic_cast<OutStream *>(stream))
  , decoder(0), encoder(0)
{
  this->properties()->addProperty(sync);
}

StreamInterface::~StreamInterface() {
  stop();
  cleanup();
}


bool StreamInterface::configureHook()
{
  RTT::Logger::In in( this->getName() );
  return true;
}

void StreamInterface::cleanupHook()
{
  RTT::Logger::In in( this->getName() );
}

bool StreamInterface::startHook()
{
  RTT::Logger::In in( this->getName() );

  if (inStream) {
    decoder = new UBX::Decoder(&buffer);
    if (!sync.get().empty() && !decoder->ublox()->setSync(sync.get())) {
      RTT::log( RTT::Error ) << "Illegal sync sequence: " << sync.get() << RTT::endlog();
      stopHook();
      return false;
    }
  }

  if (outStream) {
    encoder = new UBX::Encoder(outStream);
    if (!sync.get().empty() && !encoder->ublox()->setSync(sync.get())) {
      RTT::log( RTT::Error ) << "Illegal sync sequence: " << sync.get() << RTT::endlog();
      stopHook();
      return false;
    }
  }

  breakUpdate = false;
  return true;
}

void StreamInterface::stopHook()
{
  RTT::Logger::In in( this->getName() );

  delete decoder;
  delete encoder;
}

void StreamInterface::updateHook()
{
  Data::Streamable* data = 0;

  if (!decoder || !inStream) {
    RTT::log(RTT::Error) << "Stream input or decoder not initialized" << RTT::endlog();
    stop();
    return;
  }

  // this should be a blocking operation, but it is not...
  if (!inStream->wait(1000)) {
    RTT::log(RTT::Warning) << "No data received within 1 second..." << RTT::endlog();
    return;
  }

  *inStream >> buffer;

//    if (buffer.size() == 0) {
//      usleep(1000);
//      continue;
//    }

  while (decoder->find()) {
    Data::Key key(decoder->header()->classId, decoder->header()->messageId);
    if (connections.count(key)) {
      connections.at(key)->read(*decoder);
    }

    if (Data::Streamable::isValid(data = decoder->read())) {
      if (!data->getTimestamp()) data->setTimestamp();
      newData(data);
    }
  }

  if (!this->getActivity()->isPeriodic() && !inStream->eof()) {
    this->getActivity()->trigger();
  }
}

bool StreamInterface::newData(const Data::Streamable *data) {
  if (!dataPool) return false;

  Data::TypeInfo *typeInfo = data->getTypeInfo();
  Data::Key key = data->getKey();

  DataPool::Connection *connection = dataPool->find(key, typeInfo->RTTTypeInfo());
  if (!connection) {
    RTT::log( RTT::Debug ) << "Could not find matching DataPool entry for type " << typeInfo->getTypeName() << " (" << key << ")" << RTT::endlog();
    connection = dataPool->addConnection(data->getName(), typeInfo, key);
    if (!connection) return false;
  }

  connection->send(typeInfo->buildReference(const_cast<Data::Streamable *>(data)));
  return true;
}

bool StreamInterface::breakUpdateHook() {
  breakUpdate = true;
  return true;
}

ORO_CREATE_COMPONENT(StreamInterface)

} // namespace Interface
} // namespace uxvcos
