#ifndef _ROS_INTEGRATION_ROS_PLUGIN_H
#define _ROS_INTEGRATION_ROS_PLUGIN_H

#include "ros_publisher.hpp"
#include "ros_subscriber.hpp"

namespace ros_integration {
  template <typename InputType, typename OutputType>
  PublisherBase* publish(RTT::base::PortInterface *port, const std::string& topic) {
    return new Publisher<InputType,OutputType>(port, topic);
  }
}

#endif // _ROS_INTEGRATION_ROS_PLUGIN_H
