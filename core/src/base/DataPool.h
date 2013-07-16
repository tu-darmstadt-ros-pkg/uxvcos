#ifndef UXVCOS_DATAPOOL_H
#define UXVCOS_DATAPOOL_H

#include <uxvcos.h>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/types/Types.hpp>
#include <rtt/internal/Signal.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>

#include <uxvcos/Time.h>
#include <data/Streamable.h>

#include <set>

namespace uxvcos {

class UXVCOS_API DataPool : public RTT::TaskContext {
private:
  static DataPool *global;

public:
  static DataPool *Global();

  friend class Connection;
  class UXVCOS_API Connection {
  public:
    Connection(DataPool *owner, RTT::base::PortInterface *port, std::string name, Data::Key key = Data::Key());
    ~Connection();

    std::string name;
    const RTT::types::TypeInfo *typeInfo;
    Data::Key key;
    RTT::base::InputPortInterface *inputPort, *internalInputPort;
    RTT::base::OutputPortInterface *outputPort, *internalOutputPort;

    bool hasWriters() const;
    bool hasReaders() const;
    std::size_t numReaders() const;
    std::size_t numWriters() const;

  public:
    bool addPort(RTT::base::PortInterface *port);
    void removePort(RTT::base::PortInterface *port);

    const std::string& getName() const;

    RTT::base::DataSourceBase::shared_ptr getDataSource() const;

    void send(RTT::base::DataSourceBase::shared_ptr ds, bool forward = true);

    template <typename T>
    bool getLastValue(T& sample) {
      typename RTT::internal::DataSource<T>::shared_ptr ds = RTT::internal::DataSource<T>::narrow(getDataSource().get());
      if (!ds) return false;
      sample = ds->get();
      return true;
    }

  private:
    void dataOnPort(RTT::base::PortInterface *port);

    RTT::base::DataSourceBase::shared_ptr data;
    std::set<RTT::base::OutputPortInterface *> writers;
    std::set<RTT::base::InputPortInterface *> readers;

    RTT::base::DataSourceBase::shared_ptr inputSource, internalInputSource;

    DataPool *dataPool;
  };

  typedef std::vector<Connection *> Connections;
  typedef RTT::DataFlowInterface::Ports Ports;

public:
  DataPool(const std::string& name = "DataPool");
  virtual ~DataPool();

  Connection *inject(Data::Streamable *data);

  bool addComponentByName(const std::string& name);
  bool addComponent(RTT::TaskContext* comp);
  bool removeComponentByName(const std::string& name);
  bool removeComponent(RTT::TaskContext* comp);

  bool addPort(RTT::base::PortInterface* port, const std::string& name = std::string(), Data::Key key = Data::Key());
  void removePort(RTT::base::PortInterface* port);

  Connections getConnections() const;
  Ports getInputPorts() const;
  Ports getOutputPorts() const;

  typedef RTT::internal::Signal<void(Connection *)> NewDataEvent;
  NewDataEvent* getNewDataEvent();

  Connection *addConnection(RTT::base::PortInterface* port, const std::string& name, Data::Key key = 0);
  Connection *addConnection(const std::string& name, const RTT::types::TypeInfo *typeInfo, Data::Key key = 0);
  bool removeConnection(DataPool::Connection *connection);

  Connection *find(const std::string& name, const RTT::types::TypeInfo *typeInfo = 0);
  Connection *find(const Data::Key key, const RTT::types::TypeInfo *typeInfo = 0);

  RTT::base::InputPortInterface *getInputPort(const std::string& name, const RTT::types::TypeInfo *typeInfo = 0);
  RTT::base::InputPortInterface *getInputPort(const Data::Key key, const RTT::types::TypeInfo *typeInfo = 0);
  RTT::base::OutputPortInterface *getOutputPort(const std::string& name, const RTT::types::TypeInfo *typeInfo = 0);
  RTT::base::OutputPortInterface *getOutputPort(const Data::Key key, const RTT::types::TypeInfo *typeInfo = 0);

  template <typename T>
  bool getLastValue(T& sample, const std::string& name, const RTT::types::TypeInfo *typeInfo = 0) {
    Connection *connection = find(name, typeInfo);
    return (connection && connection->getLastValue<T>(sample));
  }

  uxvcos::Time getTimestamp() const { return timestamp; }
  void setTimestamp(uxvcos::Time ts) { timestamp = ts; }

protected:
  void newData(Connection* connection);
  Connections connections;

  friend class Connection;
  RTT::DataFlowInterface inputPorts;
  RTT::DataFlowInterface internalInputPorts;
  RTT::DataFlowInterface outputPorts;
  RTT::DataFlowInterface internalOutputPorts;

  uxvcos::Time timestamp;
  NewDataEvent newDataEvent;
};

} // namespace uxvcos

#endif // UXVCOS_DATAPOOL_H
