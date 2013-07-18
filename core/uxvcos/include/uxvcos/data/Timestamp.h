#ifndef DATA_TIMESTAMP_H
#define DATA_TIMESTAMP_H

#include <stdexcept>
#include <iostream>
#include "TypeInfo.h"

#include <uxvcos/uxvcos.h>

namespace Data {

class UXVCOS_API Timestamp {
public:
  typedef long long ticks;
  typedef double Seconds;

  Timestamp() : t(0) {}
  // Timestamp(const int init) : t(init) {}
  Timestamp(const ticks init) : t(init) {}
  Timestamp(const Timestamp& other) : t(other.t) {}
  explicit Timestamp(const Seconds& seconds) { fromSeconds(seconds); }
  Timestamp(unsigned long sec, unsigned long nsec) { fromSeconds(Seconds(sec) + Seconds(nsec)*1e-9); }

  operator ticks() const { return t; }
  operator Seconds() const { return toSeconds(); }
  operator bool() const { return (t != 0); }
  
  Timestamp& operator=(const Timestamp& other)  { t = other.t; return *this; }
  bool operator==(const Timestamp& other) const { return t == other.t; }
  bool operator!=(const Timestamp& other) const { return t != other.t; }
  bool operator<(const Timestamp& other) const  { return t < other.t; }
  bool operator>(const Timestamp& other) const  { return t > other.t; }
  bool operator<=(const Timestamp& other) const { return t <= other.t; }
  bool operator>=(const Timestamp& other) const { return t >= other.t; }

  Seconds operator-(const Timestamp& other) const      { return ticks2seconds(t - other.t); }
  Timestamp operator+(const Seconds& seconds) const   { return Timestamp(t + seconds2ticks(seconds)); }

  void reset() { t = 0; }

  Seconds toSeconds() const {
    return t ? ticks2seconds(t - reference) : 0.0;
  }

  void fromSeconds(Seconds seconds) {
    t = seconds2ticks(seconds) + reference;
  }
  
  Seconds getSeconds(const Timestamp& other) {
    Seconds result;
    result = (t != 0 && other.t != 0) ? ticks2seconds(other.t - t) : 0.0;
    t = other.t;
    return result;
  }

  static Timestamp fromStreamable(const Data::Streamable& data);
  static ticks getTicks();

  static Timestamp getTimestamp() {
    return Timestamp(getTicks());
  }

  static void setReference(ticks ref) {
    reference = ref;
  }
  
  static Timestamp now() {
    return getTimestamp();
  }

#ifdef OROCOS_TARGET
  static inline Seconds ticks2seconds(const ticks ticks) { return Seconds(RTT::os::TimeService::ticks2nsecs(ticks)) / 1e9; }
  static inline ticks seconds2ticks(const Seconds seconds) { return RTT::os::TimeService::nsecs2ticks(RTT::os::TimeService::nsecs(seconds * 1e9)); }
#elif USE_XENOMAI
  static inline Seconds ticks2seconds(const ticks ticks) { return Seconds(rt_timer_tsc2ns(ticks)) / 1e9; }
  static inline ticks seconds2ticks(const Seconds seconds) { return ticks(rt_timer_ns2tsc(seconds * 1e9)); }
#elif SYSTEM_UNIX
  static inline Seconds ticks2seconds(const ticks ticks) { return Seconds(ticks) / 1e9; }
  static inline ticks seconds2ticks(const Seconds seconds) { return ticks(seconds * 1e9); }
#elif SYSTEM_WIN32
  static inline Seconds ticks2seconds(const ticks ticks) { return Seconds(ticks) / 1e9; }
  static inline ticks seconds2ticks(const Seconds seconds) { return ticks(seconds * 1e9); }
#endif

private:
  ticks t;
  static ticks reference;
};

static inline std::ostream& operator<<(std::ostream& os, const Timestamp& timestamp) {
  return os << timestamp.toSeconds();
}

static inline std::istream& operator>>(std::istream& is, Timestamp& timestamp) {
  Timestamp::Seconds seconds;
  if (is >> seconds) timestamp.fromSeconds(seconds);
  return is;
}

} // namespace Data

#ifdef OROCOS_TARGET
  #include <rtt/os/TimeService.hpp>
  Data::Timestamp::ticks Data::Timestamp::getTicks() {
    return RTT::os::TimeService::Instance()->getTicks();
  }
#endif

#endif // DATA_TIMESTAMP_H
