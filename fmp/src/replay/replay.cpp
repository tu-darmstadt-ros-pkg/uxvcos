//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>
#include <rosbag/message_instance.h>

#include <topic_tools/shape_shifter.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Header.h>

#include <options/options.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#define ROS_LOG_THROTTLE_WALL(rate, level, name, ...) \
  do \
  { \
    ROSCONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::WallTime now = ::ros::WallTime::now(); \
    if (ROS_UNLIKELY(enabled) && ROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

#define ROS_INFO_THROTTLE_WALL(rate, ...) ROS_LOG_THROTTLE_WALL(rate, ::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

using namespace uxvcos;

namespace fmp
{

class Replay {
private:
  std::list< boost::shared_ptr<rosbag::Bag> > bags_;
  rosbag::View view_;
  rosbag::View::iterator iterator_;

  ros::NodeHandle nh_;
  std::map<std::string,ros::Publisher> publishers_;

  ros::Subscriber trigger_;
  rosgraph_msgs::Clock clock_;
  ros::Publisher clock_publisher_;
  ros::Time last_published_clock_;

  ros::Time start_time_;
  ros::Time stop_time_;
  ros::Duration start_skip_;
  ros::WallTime start_wall_;
  double rate_;

  static const boost::gregorian::date DATE_MAX;

public:
  Replay()
    : start_time_(ros::TIME_MIN)
    , stop_time_(ros::TIME_MAX)
    , rate_(1.0)
  {
  }

  ~Replay()
  {
  }

  void open(const std::string& sensor_bag, const std::string& processed_bag = std::string())
  {
    const rosbag::Bag& sensor(**bags_.insert(bags_.end(), boost::make_shared<rosbag::Bag>(sensor_bag)));
    ros::V_string sensor_topics;
    sensor_topics.push_back("raw_imu");
    sensor_topics.push_back("fix");
    sensor_topics.push_back("fix_velocity");
    sensor_topics.push_back("magnetic");
    sensor_topics.push_back("altimeter");
    sensor_topics.push_back("fmp");
    sensor_topics.push_back("supply");
    view_.addQuery(sensor, rosbag::TopicQuery(sensor_topics));

    ros::V_string processed_topics;
    processed_topics.push_back("state");
    processed_topics.push_back("imu");
    processed_topics.push_back("global");
    processed_topics.push_back("euler");

    if (processed_bag.empty()) {
      view_.addQuery(sensor, rosbag::TopicQuery(processed_topics), start_time_, stop_time_);
    } else if (processed_bag != "-") {
      const rosbag::Bag& processed(**bags_.insert(bags_.end(), boost::make_shared<rosbag::Bag>(processed_bag)));
      view_.addQuery(processed, rosbag::TopicQuery(processed_topics), start_time_, stop_time_);
    }

    clock_publisher_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 100);

    iterator_ = view_.begin();
    ROS_INFO("Opened bag file with %lu connections.", view_.getConnections().size());
  }

  void close()
  {
    iterator_ = view_.end();
    bags_.clear();
  }

  void step()
  {
    while(ros::ok() && (iterator_ != view_.end())) {
      const rosbag::MessageInstance& msg = *iterator_;

      // skip all messages with t < begin + start_skip_
      if (!start_skip_.isZero() && msg.getTime() < view_.getBeginTime() + start_skip_) {
        ++iterator_;
        ros::spinOnce();
        continue;
      }

      // set logged start time / add date to user-specified start time
      if (start_time_ == ros::TIME_MIN) {
        start_time_ = msg.getTime();
      } else if (start_time_.toBoost().date() == ros::Time().toBoost().date()) {
        start_time_ = ros::Time::fromBoost(boost::posix_time::ptime(msg.getTime().toBoost().date(), start_time_.toBoost().time_of_day()));
      }

      // add date to logged stop time
      if (stop_time_ != ros::TIME_MAX && stop_time_.toBoost().date() == DATE_MAX) {
        stop_time_ = ros::Time::fromBoost(boost::posix_time::ptime(msg.getTime().toBoost().date(), stop_time_.toBoost().time_of_day()));
      }

      // skip all messages with t < start_time_
      if (msg.getTime() < start_time_) {
        // std::cout << "Skipping message at t = " << msg.getTime() << std::endl;
        ros::Duration current(msg.getTime() - view_.getBeginTime());
        ros::Duration total(view_.getEndTime() - view_.getBeginTime());
        ROS_INFO_THROTTLE_WALL(1.0, "Skipping messages at t = %.1fs (%.1f%%)...", current.toSec(), current.toSec() / total.toSec() * 100.0);

        ++iterator_;
        ros::spinOnce();
        continue;
      }

      // finish if t >= stop_time_
      if (msg.getTime() >= stop_time_) {
        iterator_ = view_.end();
        break;
      }

      // set wall start time
      if (start_wall_.isZero()) start_wall_ = ros::WallTime::now();

      // spin if replay is too fast
      if (!trigger_) {
        setClock(start_time_ + ros::Duration((ros::WallTime::now() - start_wall_).toSec() * rate_));

        if (msg.getTime() > getClock()) {
          ros::WallDuration(0.01).sleep();
          ros::spinOnce();
          continue;
        }

      } else {
        if (rate_ > 0.0) {
          ros::WallDuration diff(start_wall_ + ros::WallDuration((msg.getTime() - start_time_).toSec() / rate_) - ros::WallTime::now());
          if (diff > ros::WallDuration(0, 0)) diff.sleep();
        }

        if (getClock().isZero() && msg.getTopic() == "raw_imu") setClock(msg.getTime());

        if (!getClock().isZero() && (msg.getTime() > getClock() + ros::Duration(0.015))) {
          return;
        }
      }

      // publish data
      ros::Publisher& publisher = publishers_[msg.getTopic()];
      if (!publisher) {
        ros::AdvertiseOptions ops = rosbag::createAdvertiseOptions(msg, 100);
        publisher = nh_.advertise(ops);
        ROS_INFO("Advertising %s...", ops.topic.c_str());
        ros::WallDuration(0.2).sleep();
      }

      publisher.publish(msg);
      ++iterator_;
    }

    if (iterator_ == view_.end()) {
      ROS_INFO("Finished replay.");
      ros::shutdown();
    }
  }

  void addTrigger(const std::string& topic) {
    trigger_ = nh_.subscribe<topic_tools::ShapeShifter>("state", 100, boost::bind(&Replay::trigger, this, _1));
  }

  void trigger(const boost::shared_ptr<topic_tools::ShapeShifter const>& message) {
    if (message->getDataType() == "rosgraph_msgs/Clock") {
      setClock(message->instantiate<rosgraph_msgs::Clock>()->clock);

    } else {
      // assume the message starts with a header
      boost::shared_ptr<topic_tools::ShapeShifter> header_message = boost::const_pointer_cast<topic_tools::ShapeShifter>(message);
      header_message->morph(ros::message_traits::md5sum<std_msgs::Header>(), ros::message_traits::datatype<std_msgs::Header>(), ros::message_traits::definition<std_msgs::Header>(), std::string());
      std_msgs::HeaderPtr header = header_message->instantiate<std_msgs::Header>();
      if (header) {
        setClock(header->stamp);
      }
    }
    // std::cout << "Trigger at t = " << getClock() << std::endl;

    step();
  }

  void setRate(double rate) {
    rate_ = rate;
  }

  void setClock(const ros::Time& clock)
  {
    if (clock < clock_.clock) return;

    ros::Duration diff(clock - last_published_clock_);
    clock_.clock = clock;
    if (diff >= ros::Duration(0, 100000)) {
      clock_publisher_.publish(clock_);
      last_published_clock_ = clock_.clock;
    }

    ros::Duration current(getClock() - view_.getBeginTime());
    ros::Duration total(view_.getEndTime() - view_.getBeginTime());
    ROS_INFO_THROTTLE_WALL(1.0, "Replay t = %.1fs (%.1f%%)...", current.toSec(), current.toSec() / total.toSec() * 100.0);
    // ROS_INFO("Replay t = %.1fs (%.1f%%)...", current.toSec(), current.toSec() / total.toSec() * 100.0);
  }

  const ros::Time& getClock() const
  {
    return clock_.clock;
  }

  void setStartTime(const boost::posix_time::time_duration& time)
  {
    start_time_ = ros::Time::fromBoost(time);
  }

  void setStopTime(const boost::posix_time::time_duration& time)
  {
    stop_time_ = ros::Time::fromBoost(time);
    if (stop_time_.toBoost().date() == ros::Time().toBoost().date()) {
      stop_time_.sec += ros::Time::fromBoost(boost::posix_time::ptime(DATE_MAX)).sec;
    }
  }

  void setStartSkip(const ros::Duration& skip)
  {
    start_skip_ = skip;
  }
};

const boost::gregorian::date Replay::DATE_MAX(2100, 1, 1);

} // namespace fmp

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fmp_replay");
  Options::add_options()("sensor", Options::value<std::string>()->required(), "Bag file with sensor data");
  Options::add_options()("processed", Options::value<std::string>(), "Bag file with processed data");
  Options::add_options()("trigger,t", Options::value<std::string>(), "Topic which triggers data to be read");
  Options::add_options()("rate,r", Options::value<double>()->default_value(1.0), "Replay rate");
  Options::add_options()("skip", Options::value<double>()->implicit_value(0.0), "Skip s seconds at the beginning of the bag file");
  Options::add_options()("start", Options::value<std::string>()->implicit_value(std::string()), "Time of the earliest message to process");
  Options::add_options()("stop", Options::value<std::string>()->implicit_value(std::string()), "Time of the latest message to process");
  Options::add_options()("paused", Options::value<bool>()->implicit_value(false), "Start in paused mode");
  Options::positional().add("sensor", 1).add("processed", 1);

  fmp::Replay replay;

  Options::parse(argc, argv);
  if (Options::variables("sensor").empty()) {
    std::cout << "Usage:" << std::endl;
    Options::help();
    return 1;
  }
  if (!Options::variables("trigger").empty()) {
    std::string trigger_topic = Options::variables("trigger").as<std::string>();
    std::cout << "Trigger:    " << trigger_topic << std::endl;
    replay.addTrigger(trigger_topic);
  }
  if (!Options::variables("rate").empty()) {
    double rate = Options::variables("rate").as<double>();
    std::cout << "Rate:       " << rate << std::endl;
    replay.setRate(rate);
  }
  if (!Options::variables("start").empty()) {
    try {
      boost::posix_time::time_duration start = boost::posix_time::duration_from_string(Options::variables("start").as<std::string>());
      std::cout << "Start time: " << start << std::endl;
      replay.setStartTime(start);
    } catch(std::exception& e) {
    }
  }
  if (!Options::variables("stop").empty()) {
    try {
      boost::posix_time::time_duration stop = boost::posix_time::duration_from_string(Options::variables("stop").as<std::string>());
      std::cout << "Stop time:  " << stop << std::endl;
      replay.setStopTime(stop);
    } catch(std::exception& e) {
    }
  }
  if (!Options::variables("skip").empty()) {
    ros::Duration skip(Options::variables("skip").as<double>());
    std::cout << "Skip:       " << skip << std::endl;
    replay.setStartSkip(skip);
  }

  replay.open(Options::variables("sensor").as<std::string>(), Options::variables("processed").empty() ? std::string() : Options::variables("processed").as<std::string>());
  replay.step();
  ros::spin();
  return 0;
}
