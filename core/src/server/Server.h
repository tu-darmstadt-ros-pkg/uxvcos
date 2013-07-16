#ifndef UXVCOS_SERVER_H
#define UXVCOS_SERVER_H

#include <uxvcos.h>

#include <system/Socket.h>
#include <stream/ubx/Stream.h>
#include <stream/Buffer.h>

#include <base/PortListener.h>
#include <base/DataPool.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Activity.hpp>

#include <boost/shared_ptr.hpp>

#include <connector/Connector.h>

namespace uxvcos {

class UXVCOS_EXPORT Server : public RTT::TaskContext, public OutConnector
{
  public:
    Server(const std::string& name = "Server", DataPool *dataPool = 0, int capacity = 10);
    virtual ~Server();

    static int setup();

    DataPool *data() const { return dataPool; }

  protected:
    bool configureHook();
    bool startHook();
    void updateHook();
    bool breakUpdateHook();
    void stopHook();
    void cleanupHook();

    bool send(RTT::base::DataSourceBase::shared_ptr dsb);
    bool send(const Data::Streamable& message);

  protected:
    RTT::Property<std::string> type;
    RTT::Property<std::string> destinationHost;
    RTT::Property<unsigned int> destinationPort;
    RTT::Property<unsigned int> localPort;

  protected:
    class ClientConnection : public RTT::Activity
    {
      public:
        typedef boost::shared_ptr<ClientConnection> shared_ptr;

        ClientConnection(Server *server, const System::Socket& s)
          : RTT::Activity(0, "ClientConnection")
          , in(65536)
          , server(server)
          , socket(s)
        {
          // encoder = new UBX::Encoder(&socket);
          decoder = new UBX::Decoder(&in);
        }

        ~ClientConnection()
        {
          stop();

          // delete encoder;
          delete decoder;
        }
        
        bool alive() const {
          return socket.isValid();
        }
        
        bool send(const void* data, unsigned long size) {
          return socket.send(data, size);
        }

        virtual bool initialize();
        virtual void loop();
        virtual bool breakLoop();
        virtual void finalize();

      public:
        Buffer<> in;
        // UBX::Encoder *encoder;
        UBX::Decoder *decoder;
        
      private:
        Server *server;
        System::Socket socket;
    };
    typedef std::vector<ClientConnection::shared_ptr> Connections;
    Connections connections;

    void addConnection(ClientConnection *connection);
    void removeConnection(ClientConnection *connection);
    Connections::iterator removeConnectionImpl(Connections::iterator it);

    class SocketAcceptor : public RTT::Activity
    {
      public:
        SocketAcceptor(Server *server, const System::Socket& socket)
          : RTT::Activity(0, "SocketAcceptor")
          , server(server)
          , addConnection(server->getOperation("addConnection"))
          , socket(socket)
          , acceptSocket(socket.getSocketType())
        {}

        ~SocketAcceptor() {
          stop();
        }
     
        virtual bool initialize();
        virtual void loop();
        virtual bool breakLoop();
        virtual void finalize();
        
        const System::Socket& getAcceptSocket(System::NetworkAddress& address) const {
          address = this->address;
          return acceptSocket;
        }
        
      private:
        Server *server;
        RTT::OperationCaller<void(ClientConnection *)> addConnection;

        System::Socket socket;
        System::Socket acceptSocket;
        System::NetworkAddress address;
    };

    System::Socket *socket;
    SocketAcceptor *acceptor;

    Buffer<> sendBuffer;
    UBX::Encoder encoder;

    DataPool *dataPool;
    PortListener portListener;
};
} // namespace uxvcos

#endif // UXVCOS_SERVER_H
