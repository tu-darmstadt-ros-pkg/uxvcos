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

#ifndef UXVCOS_SENSOR_SENSOR_H
#define UXVCOS_SENSOR_SENSOR_H

#include <uxvcos/Module.h>

#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSources.hpp>
#include <rtt/Logger.hpp>

#include <uxvcos/Time.h>
#include <uxvcos/data/Streamable.h>
#include <boost/shared_ptr.hpp>
#include <boost/utility/enable_if.hpp>

#ifdef ROS_PACKAGE_NAME
  #include <ros/message_traits.h>
  #include <std_msgs/Header.h>
#endif

namespace uxvcos {
namespace Sensors {

#ifdef ROS_PACKAGE_NAME
  typedef std_msgs::Header::_seq_type sequence_t;
  typedef std_msgs::Header Header;
#else
  typedef std::size_t sequence_t;
  struct Header {
    sequence_t seq;
    uxvcos::Time stamp;
    std::string frame_id;
    Header() : timestamp(), seq(), frame_id {}
  }
#endif
typedef boost::shared_ptr<Header> HeaderPtr;

namespace {

#ifdef ROS_PACKAGE_NAME
  using ros::message_traits::HasHeader;
#else
  template <typename T> struct HasHeader : public boost::false_type {}
#endif

  struct null_deleter
  {
    void operator()(void const *) const {}
  };

  template<typename T, class Enabled = void> struct detail {
    static const uxvcos::Time& updateTimestamp(T *obj, const uxvcos::Time& new_timestamp) { return new_timestamp; }
    static const Header& updateHeader(T *obj, const Header& new_header) { return new_header; }
    static sequence_t incrementSequence(T *obj, const sequence_t& old_value) { return 0; }
    static HeaderPtr get(T *obj) { return HeaderPtr(new Header()); }
    static void set(T* obj, const Header& header) {}
  };

  // specialization for Streamables
  template<typename T> struct detail<T, typename boost::enable_if<boost::is_base_of<Data::Streamable,T> >::type> {
    static const uxvcos::Time& updateTimestamp(T *obj, const uxvcos::Time& new_timestamp) {
      Data::Streamable* streamable = static_cast<Data::Streamable *>(obj);
      if (!new_timestamp.isZero()) streamable->setTimestamp(new_timestamp);
      return streamable->getTimestamp();
    }
    static const Header& updateHeader(T *obj, const Header& new_header) { return new_header; }
    static sequence_t incrementSequence(T *obj, const sequence_t& old_value) { return old_value + 1; }
    static HeaderPtr get(T *obj) { return HeaderPtr(new Header()); }
    static void set(T* obj, const Header& new_header) { updateTimestamp(obj, new_header.stamp); }
  };

#ifdef ROS_PACKAGE_NAME
  // specialization for ROS
  template<typename T> struct detail<T, typename boost::enable_if<HasHeader<T> >::type> {
    static const uxvcos::Time& updateTimestamp(T *obj, const uxvcos::Time& new_timestamp) {
      Header* header = ros::message_traits::header(*obj);
      if (!new_timestamp.isZero()) header->stamp = new_timestamp;
      return header->stamp;
    }

    static const Header& updateHeader(T *obj, const Header& new_header) {
      Header* header = ros::message_traits::header(*obj);
      if (header == &new_header) return *header;
      if (new_header.seq)               header->seq = new_header.seq;
      if (!new_header.frame_id.empty()) header->frame_id = new_header.frame_id;
      updateTimestamp(obj, new_header.stamp);
      return *header;
    }

    static sequence_t incrementSequence(T *obj, const sequence_t& old_value) {
      Header* header = ros::message_traits::header(*obj);
      header->seq = old_value + 1;
      return header->seq;
    }

    static HeaderPtr get(T *obj) { return HeaderPtr(ros::message_traits::header(*obj), null_deleter()); }

    static void set(T* obj, const Header& new_header) {
      Header* header = ros::message_traits::header(*obj);
      *header = new_header;
    }
  };
#endif // ROS_PACKAGE_NAME

  template<typename T> struct HeaderHelper {
    static const uxvcos::Time& updateTimestamp(T *obj, const uxvcos::Time& new_timestamp) {
      return detail<T>::updateTimestamp(obj, new_timestamp);
    }

    static const Header& updateHeader(T *obj, const Header& new_header) {
      return detail<T>::updateHeader(obj, new_header);
    }

    static sequence_t incrementSequence(T *obj, const sequence_t& old_value) {
      return detail<T>::incrementSequence(obj, old_value);
    }
    
    static HeaderPtr get(T *obj) {
      return detail<T>::get(obj);
    }
    
    static void set(T* obj, const Header& new_header) {
      detail<T>::set(obj, new_header);
    }
  };
}

template <typename T>
class Sensor : public Module
{
public:
  typedef T ValueType;

  Sensor(RTT::TaskContext* parent, const std::string& name, const std::string& port_name = "", const std::string& description = "")
    : Module(parent, name, description)
    , output(port_name.empty() ? name : port_name)
//    , data(dataSource.set())
    , header(HeaderHelper<T>::get(&data))
  {
    this->addPort(output).doc("Output of sensor " + name);
    parent->addPort(output);

    this->addProperty("frame_id", header->frame_id).doc("The frame_id of the " + name + " sensor");
  }

  virtual const ValueType& get() const {
    return data;
  }

  virtual ValueType& set() {
    return data;
  }

  virtual void Set(const ValueType& newValue, uxvcos::Time timestamp = uxvcos::Time()) {
    data = newValue;
    updated(timestamp);
  }

  void updated(const uxvcos::Time& new_timestamp = uxvcos::Time()) {
//     RTT::log( RTT::RealTime ) << this->getName() << ": " << data << RTT::endlog();
    this->header->stamp = HeaderHelper<ValueType>::updateTimestamp(&data, new_timestamp);
    this->header->seq   = HeaderHelper<ValueType>::incrementSequence(&data, this->header->seq);
    output.write(data);
    updateHook();
  }
  
  void updated(const Header& header) {
//     RTT::log( RTT::RealTime ) << this->getName() << ": " << data << RTT::endlog();
    (*this->header) = HeaderHelper<ValueType>::updateHeader(&data, header);
    output.write(data);
    updateHook();
  }

  virtual void updateHook() {}

  const uxvcos::Time& getTimestamp() const {
    return header->stamp;
  }

  const sequence_t& getSequence() const {
    return header->seq;
  }
  
  const std::string& getFrameId() const {
    return header->frame_id;
  }

  virtual std::ostream& operator>>(std::ostream& os) const {
    return os << data << " (t = " << getTimestamp() << ")";
  }

  virtual RTT::Logger& log() const {
    return RTT::log() << data << " (t = " << getTimestamp() << ")";
  }

public:
  RTT::OutputPort<ValueType> output;
  // typename RTT::internal::ValueDataSource<ValueType>::reference_t data;
  ValueType data;

protected:
  // typename RTT::internal::ValueDataSource<ValueType> dataSource;
  HeaderPtr header;
};


template <typename R, typename T>
class RawSensor : public Sensor<T>
{
public:
  using Sensor<T>::ValueType;
  typedef R RawType;

  RawSensor(RTT::TaskContext* parent, const std::string& name, const std::string& port_name = "", const std::string& description = "")
    : Sensor<T>(parent, name, port_name, description)
    , rawInput("in_" + (port_name.empty() ? name : port_name))
    , rawOutput("raw_" + (port_name.empty() ? name : port_name))
    // , raw(rawDataSource.set())
  {
    this->addPort(rawInput).doc("Raw Input values of sensor " + name);
    this->addPort(rawOutput).doc("Raw Output values of sensor " + name);

    parent->ports()->addLocalEventPort(rawInput, boost::bind(&RawSensor<R,T>::dataOnPort, this, _1));

//    if (!rawInput.getNewDataOnPortEvent()->connect(boost::bind(&RawSensor<R,T>::dataOnPort, this, _1)))
//      RTT::log( RTT::Error ) << "could not connect to newDataOnPortEvent of port " << rawInput.getName() << RTT::endlog();
//    else
//      RTT::log( RTT::Debug ) << "connected to newDataOnPortEvent of port " << rawInput.getName() << RTT::endlog();

    // initialize raw header
    HeaderHelper<RawType>::set(&raw, *(this->header));
  }
  virtual ~RawSensor() {}

  using Sensor<T>::Set;
  virtual void Set(const RawType& rawData, uxvcos::Time timestamp = uxvcos::Time()) {
    raw = rawData;
    updatedRaw(timestamp);
  }

  virtual bool updatedRaw(uxvcos::Time timestamp = uxvcos::Time()) {
    this->header->stamp = HeaderHelper<RawType>::updateTimestamp(&raw, timestamp);
    this->header->seq   = HeaderHelper<RawType>::incrementSequence(&raw, this->header->seq);

    // rawDataSource.updated();
    rawOutput.write(raw);

    if (!convert(raw)) return false;
    this->updated(*(this->header));
    return true;
  }

  virtual bool convert(const RawType& rawData) {
    return false;
  }

  void dataOnPort(RTT::base::PortInterface *)
  {
    RawType rawData;
    if (rawInput.read(rawData) != RTT::NewData) return;
    Set(rawData);
  }

  virtual std::ostream& operator>>(std::ostream& os) const {
    os << raw << " --> ";
    return this->Sensor<T>::operator >>(os);
  }

  virtual RTT::Logger& log() const {
    RTT::log() << raw << " --> ";
    return this->Sensor<T>::log();
  }

public:
  RTT::InputPort<RawType> rawInput;
  RTT::OutputPort<RawType> rawOutput;
  // typename RTT::internal::ValueDataSource<RawType>::reference_t raw;
  RawType raw;

protected:
  // typename RTT::internal::ValueDataSource<RawType> rawDataSource;
};

template <typename T>
static inline std::ostream& operator<<(std::ostream& os, const Sensor<T>& sensor) {
  return sensor >> os;
}

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_SENSOR_H
