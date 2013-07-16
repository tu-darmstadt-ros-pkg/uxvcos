#ifndef _ROS_INTEGRATION_SUBSCRIBER_HPP
#define _ROS_INTEGRATION_SUBSCRIBER_HPP

#include <rtt/base/ChannelElement.hpp>
#include <rtt/base/DataObject.hpp>
#include <rtt/internal/DataSource.hpp>

#include "ros_msg_transporter.hpp"

namespace ros_integration {

  template <typename InputType, typename OutputType>
  bool toROS(const InputType& input, OutputType& output) {
    return false;
  }

  class PublisherBase {
  public:
    virtual RTT::base::PortInterface *getSourcePort() const { return port; }
    virtual RTT::base::OutputPortInterface* getOutputPort() { return 0; }
    virtual std::string getTopic() const { return topic; }

  protected:
    PublisherBase(RTT::base::PortInterface *port, const std::string& topic)
      : port(port), topic(topic)
    {}
    virtual ~PublisherBase() {}

    RTT::base::PortInterface *port;
    std::string topic;
  };

  template <typename InputType, typename OutputType>
  class Publisher : public PublisherBase, public RTT::base::ChannelElement<OutputType> {
  public:
    typedef typename RTT::base::ChannelElement<OutputType>::reference_t reference_t;

    Publisher(RTT::base::PortInterface *port, const std::string& topic)
      : PublisherBase(port, topic)
      , inputPort(0)
      , clonedPort(0)
      , outputPort(0)
      , status(RTT::NoData)
    {
      inputPort = dynamic_cast<RTT::InputPort<InputType> *>(port);
      if (!inputPort) {
        inputPort = dynamic_cast<RTT::InputPort<InputType> *>(port->antiClone());
        clonedPort = inputPort;
        port->connectTo(inputPort);
      }
      if (!inputPort) throw std::runtime_error("Incorrect type for " + port->getName());
      connect();
    }

    Publisher(RTT::OutputPort<InputType> *senderPort, const std::string& topic)
      : PublisherBase(senderPort, topic)
      , inputPort(static_cast<RTT::InputPort<InputType> *>(senderPort->antiClone()))
      , clonedPort(inputPort)
      , outputPort(0)
      , status(RTT::NoData)
    {
      senderPort->connectTo(inputPort);
      connect();
    }

    Publisher(RTT::InputPort<InputType> *inputPort, const std::string& topic)
      : PublisherBase(inputPort, topic)
      , inputPort(inputPort), clonedPort(0)
      , outputPort(0)
      , status(RTT::NoData)
    {
      connect();
    }

    void connect()
    {
      RTT::ConnPolicy policy(RTT::ConnPolicy::data());
      policy.name_id = topic;
      this->setOutput(new RosPubChannelElement<OutputType>(inputPort, policy));
      inputPort->getNewDataOnPortEvent()->connect(boost::bind(&Publisher<InputType,OutputType>::dataOnPort, this, _1));
    }

    virtual ~Publisher() {
      if (clonedPort) delete clonedPort;
    }

    virtual void dataOnPort(RTT::base::PortInterface *port) {
      inputPort->read(input);
      if (!toROS(input, output)) return;
      status = RTT::NewData;
      this->signal();

      if (outputPort)
        outputPort->write(output);
    }

    virtual RTT::FlowStatus read(reference_t sample) {
      if (status == RTT::NoData) return status;
      sample = output;

      if (status == RTT::NewData) {
        status = RTT::OldData;
        return RTT::NewData;
      }

      return status;
    }

    virtual RTT::base::OutputPortInterface* getOutputPort() {
      if (outputPort == 0)
        outputPort = new RTT::OutputPort<OutputType>(inputPort->getName());
      return outputPort;
    }

  private:
    RTT::InputPort<InputType> *inputPort;
    RTT::base::PortInterface *clonedPort;
    RTT::OutputPort<OutputType> *outputPort;
    RTT::FlowStatus status;

    InputType input;
    OutputType output;
  };

} // namespace rtt_integration


#endif // PUBLISHER_HPP
