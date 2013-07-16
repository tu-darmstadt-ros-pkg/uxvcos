#ifndef ROS_INTEGRATION_TASKCONTEXT_HPP
#define ROS_INTEGRATION_TASKCONTEXT_HPP

#include <ros/ros.h>
#include <rtt/TaskContext.hpp>

namespace ros_integration {

class TaskContext : public RTT::TaskContext {
public:
  typedef ros::Publisher Publisher;
  typedef ros::Subscriber Subscriber;

public:
  TaskContext(const std::string& name, RTT::ExecutionEngine* parent, RTT::TaskContext::TaskState initial_state = Stopped)
    : RTT::TaskContext(name, parent, initial_state)
  {}

  TaskContext( const std::string& name, RTT::TaskContext::TaskState initial_state = Stopped )
    : RTT::TaskContext(name, initial_state)
  {}

  virtual ~TaskContext()
  {
    subscriber.clear();
    publisher.clear();
  }

  template <typename M>
  Publisher* addPublisher(const std::string& topic, uint32_t queue_size, bool latch=false) {
    publisher[topic] = nh.advertise<M>(topic, queue_size, latch);
    return &publisher[topic];
  }

  Publisher* getPublisher(const std::string& topic) {
    return &publisher[topic];
  }

  template <typename M>
  Subscriber* addSubscriber(const std::string &topic, uint32_t queue_size, void(*fp)(M), const ros::TransportHints &transport_hints = ros::TransportHints()) {
    subscriber[topic] = nh.subscribe<M>(topic, queue_size, fp, transport_hints);
    return &subscriber[topic];
  }

  template <typename M, typename T>
  Subscriber* addSubscriber(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj, const ros::TransportHints &transport_hints = ros::TransportHints()) {
    subscriber[topic] = nh.subscribe<M>(topic, queue_size, fp, obj, transport_hints);
    return &subscriber[topic];
  }

  Subscriber* getSubscriber(const std::string& topic) {
    return &subscriber[topic];
  }

  virtual void updateHook() {
  }

protected:
  ros::NodeHandle nh;
  std::map<std::string, Publisher> publisher;
  std::map<std::string, Subscriber> subscriber;
};

} // namespace ros_integration

#endif // ROS_INTEGRATION_TASKCONTEXT_HPP
