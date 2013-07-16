#ifndef _ROS_INTEGRATION_SUBSCRIBER_HPP
#define _ROS_INTEGRATION_SUBSCRIBER_HPP

namespace ros_integration {
  class Subscriber;

  struct forwarder {
    virtual bool publish(RTT::base::InputPortInterface *inputPort, ros::Publisher& publisher) = 0;
    virtual bool read(const ros::Message& message, RTT::base::OutputPortInterface *outputPort) = 0;
  };

  template <typename InputType, typename OutputType>
  class converter : public forwarder {
  public:
    virtual bool operator()(const InputType& input, OutputType& output);

    virtual bool publish(RTT::base::InputPortInterface *inputPort, ros::Publisher& publisher) {
      RTT::base::AssignableDataSource<InputType>::shared_ptr ds = boost::dynamic_pointer_cast<RTT::base::AssignableDataSource<InputType> >(inputPort->getDataSource());
      if (!ds) return false;

      RTT::OutputPort<OutputType> *typed = dynamic_cast<RTT::OutputPort<OutputType> *>(outputPort);
      if (!typed) return false;

      OutputType outputData;
      if (!operator()(ds->rvalue(), outputData)) return false;

      typed.write(outputData);
      return true;
    }
  };

  class Subscriber {
  public:
    template <typename InputType, typename OutputType>
    Subscriber(RTT::OutputPort<OutputType> *outputPort, const std::string& topic)
      : inputPort(0), outputPort(outputPort)
      , topic(topic)
      , converterImpl(new converter<InputType,OutputType>)
    {
      RTT::ConnPolicy policy(RTT::ConnPolicy::data());
      policy.transport = ORO_PROTOCOL_ID;
      policy.name_id = topic;

      inputPort = new RTT::InputPort<InputType>(outputPort->getName());
      outputPort->createStream(policy);

      handle = inputPort->getNewDataOnPortEvent()->connect(boost::bind(&Subscriber::forward, this, _1));
    }

    virtual ~Subscriber() {
      handle.disconnect();
      delete inputPort;
      delete converterImpl;
    }

    virtual void forward(RTT::base::PortInterface *port) {
      forwarderImpl->forward(inputPort, outputPort);
    }

    std::string getTopic() const { return topic; }

  private:
    RTT::base::InputPortInterface *inputPort;
    RTT::base::OutputPortInterface *outputPort;
    std::string topic;
    converter *converterImpl;
    RTT::Handle handle;
  };

} // namespace ros_integration

#endif // _ROS_INTEGRATION_SUBSCRIBER_HPP
