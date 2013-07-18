#ifndef UXVCOS_INTERFACE_STREAMINTERFACE_H
#define UXVCOS_INTERFACE_STREAMINTERFACE_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>

#include <ros/serialization.h> // must be included before <stream/Stream.h>
#include <std_msgs/Header.h>

#include <uxvcos/ModuleContainer.h>
#include <uxvcos/DataPool.h>
#include <uxvcos/stream/Stream.h>
#include <uxvcos/stream/ubx/Stream.h>

#include <ros/node_handle.h>

#include "BaseInterface.h"

namespace uxvcos {
namespace Interface {

template <typename T, class Enabled = void>
struct StreamSerializer {
  inline static OutStream& write(OutStream& stream, const T& data)
  {
    stream.fail();
    return stream;
  }

  inline static InStream& read(InStream& stream, T& data)
  {
    stream.fail();
    return stream;
  }

  inline static uint32_t serializedLength(const T& data)
  {
    return 0;
  }
};

template <typename T>
struct StreamSerializer<T, typename boost::enable_if<ros::message_traits::IsMessage<T> >::type> {
  inline static OutStream& write(OutStream& stream, const T& data)
  {
    ros::serialization::serialize(stream, data);
    return stream;
  }

  inline static InStream& read(InStream& stream, T& data)
  {
    ros::serialization::deserialize(stream, data);
    return stream;
  }

  inline static uint32_t serializedLength(const T& data)
  {
    return ros::serialization::serializationLength(data);
  }
};

class StreamConnectionBase
{
public:
  StreamConnectionBase(const std::string &name) : _name(name) {}
  virtual ~StreamConnectionBase() {}
  virtual void read(InStream& stream) = 0;

  const std::string& getName() const { return _name; }
  const Time& getTimestamp() const { return _stamp; }
  const Data::Key& getKey() const { return _key; }

  StreamConnectionBase& setName(const std::string& name) { _name = name; return *this; }
  StreamConnectionBase& setKey(const Data::Key& key) { _key = key; return *this; }

protected:
  std::string _name;
  Time _stamp;
  Data::Key _key;
};

typedef boost::shared_ptr<StreamConnectionBase> StreamConnectionPtr;
typedef std::map<Data::Key,StreamConnectionPtr> ConnectionMap;

template <typename T>
class StreamConnection : public StreamConnectionBase
{
public:
  typedef boost::function<void(const T&, const StreamConnectionBase *)> Callback;

  StreamConnection(const std::string& name, const Callback& callback) : StreamConnectionBase(name), _callback(callback) {}
  virtual ~StreamConnection() {}

  virtual void read(InStream& stream) {
    if (!stream.start()) return;
    if (StreamSerializer<T>::read(stream, _data)) {
      UBX::Decoder *decoder = dynamic_cast<UBX::Decoder *>(&stream);
      if (decoder) {
        _stamp = decoder->getTimestamp();
      } else {
        _stamp = Time::now();
      }
      _callback(_data, this);
    }
    stream.finish();
  }

private:
  T _data;
  Callback _callback;
};

class StreamInterface
  : public BaseInterface
{
public:
  StreamInterface(const std::string &name = "StreamInterface", BaseStream *stream = 0);
  virtual ~StreamInterface();

  void setDataPool(DataPool *dataPool) { this->dataPool = dataPool; }
  const DataPool *getDataPool() const  { return dataPool; }
  void setInStream(InStream *inStream) { this->inStream = inStream; }
  const InStream *getInStream() const  { return inStream; }
  void setOutStream(OutStream *outStream) { this->outStream = outStream; }
  const OutStream *getOutStream() const  { return outStream; }

  virtual bool newData(const Data::Streamable* data);

  template <typename T>
  const StreamConnectionPtr& addConnection(const Data::Key& key, const std::string& name) {
    return addConnection<T>(key, name, boost::bind(&StreamInterface::defaultCallback<T>, this, _1, _2));
  }

  template <typename T, class Object>
  const StreamConnectionPtr& addConnection(const Data::Key& key, const std::string& name, void (Object::*func)(const T&, const StreamConnectionBase *), Object *obj) {
    typename StreamConnection<T>::Callback callback =  boost::bind(func, obj, _1, _2);
    return addConnection<T>(key, name, callback);
  }

  template <typename T>
  const StreamConnectionPtr& addConnection(const Data::Key& key, const std::string& name, typename StreamConnection<T>::Callback callback) {
    StreamConnectionPtr connection(new StreamConnection<T>(name, callback));
    connection->setKey(key);
    return connections.insert(std::pair<Data::Key,StreamConnectionPtr>(key, connection)).first->second;
  }

  template <typename T>
  void defaultCallback(const T& data, const StreamConnectionBase *source) {
    if (!dataPool) return;

    if (ros::message_traits::hasHeader<T>()) {
      std_msgs::Header *header = ros::message_traits::header(data);
      if (header->stamp.isZero()) header->stamp = source->getTimestamp();
    }

    RTT::types::TypeInfo *typeInfo = RTT::types::Types()->getTypeInfo<T>();
    if (!typeInfo) {
      RTT::log( RTT::Warning ) << getName() << " received a message with unknown type" << RTT::endlog();
      return;
    }

    DataPool::Connection *connection = dataPool->find(source->getKey(), typeInfo) || dataPool->find(source->getName(), typeInfo);
    if (!connection) {
      RTT::log( RTT::Debug ) << "Could not find matching DataPool entry for type " << typeInfo->getTypeName() << " (" << source->getKey() << ")" << RTT::endlog();
      connection = dataPool->addConnection(source->getName(), typeInfo, source->getKey());
      if (!connection) return false;
    }

    connection->send(typeInfo->buildReference(const_cast<T *>(&data)));
    return;
  }

protected:
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual bool breakUpdateHook();
  virtual void stopHook();
  virtual void cleanupHook();

protected:
  RTT::Property<std::string> sync;

protected:
  DataPool *dataPool;

  InStream *inStream;
  OutStream *outStream;
  Buffer<> buffer;
  UBX::Decoder *decoder;
  UBX::Encoder *encoder;

  ConnectionMap connections;

private:
  bool breakUpdate;
};

} // namespace Interface
} // namespace uxvcos

#endif // UXVCOS_INTERFACE_STREAMINTERFACE_H
