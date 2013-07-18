//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#ifndef UXVCOS_TIME_H
#define UXVCOS_TIME_H

#include <uxvcos/uxvcos.h>

#ifdef ROS_PACKAGE_NAME
  #include <ros/time.h>
#else // ROS_PACKAGE_NAME
  #include <stdint.h>
  #include <boost/math/special_functions/round.hpp>
#endif // ROS_PACKAGE_NAME

namespace uxvcos {

#ifdef ROS_PACKAGE_NAME

typedef ros::Time Time;
typedef ros::Duration Duration;

#else // ROS_PACKAGE_NAME

  class Time;
  class Duration;

  UXVCOS_API void normalizeSecNSec(uint64_t& sec, uint64_t& nsec);
  UXVCOS_API void normalizeSecNSec(uint32_t& sec, uint32_t& nsec);
  UXVCOS_API void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec);

  template<class T, class D>
  class UXVCOS_API TimeBase
  {
  public:
    uint32_t sec, nsec;

    TimeBase() : sec(0), nsec(0) { }
    TimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
    {
      normalizeSecNSec(sec, nsec);
    }
    explicit TimeBase(double t) { fromSec(t); }
    ~TimeBase() {}
    D operator-(const T &rhs) const;
    T operator+(const D &rhs) const;
    T operator-(const D &rhs) const;
    T& operator+=(const D &rhs);
    T& operator-=(const D &rhs);
    bool operator==(const T &rhs) const;
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const;
    bool operator<(const T &rhs) const;
    bool operator>=(const T &rhs) const;
    bool operator<=(const T &rhs) const;

    double toSec()  const { return (double)sec + 1e-9*(double)nsec; };
    T& fromSec(double t) { sec = (uint32_t)floor(t); nsec = (uint32_t)boost::math::round((t-sec) * 1e9);  return *static_cast<T*>(this);}

    uint64_t toNSec() const {return (uint64_t)sec*1000000000ull + (uint64_t)nsec;  }
    T& fromNSec(uint64_t t);

    inline bool isZero() const { return sec == 0 && nsec == 0; }
    inline bool is_zero() const { return isZero(); }
//    boost::posix_time::ptime toBoost() const;

  };
  
  class UXVCOS_API Time : public TimeBase<Time, Duration>
  {
  public:
    Time()
      : TimeBase<Time, Duration>()
    {}

    Time(uint32_t _sec, uint32_t _nsec)
      : TimeBase<Time, Duration>(_sec, _nsec)
    {}

    explicit Time(double t) { fromSec(t); }

    /**
     * \brief Retrieve the current time.  If ROS clock time is in use, this returns the time according to the
     * ROS clock.  Otherwise returns the current wall clock time.
     */
    static Time now();

    /**
     * \brief Returns whether or not the current time is valid.  Time is valid if it is non-zero.
     */
    static bool isValid();

//    static Time fromBoost(const boost::posix_time::ptime& t);
//    static Time fromBoost(const boost::posix_time::time_duration& d);
  };

  /**
   * \brief Base class for Duration implementations.  Provides storage, common functions and operator overloads.
   * This should not need to be used directly.
   */
  template <class T>
  class UXVCOS_API DurationBase
  {
  public:
    int32_t sec, nsec;
    DurationBase() : sec(0), nsec(0) { }
    DurationBase(int32_t _sec, int32_t _nsec);
    explicit DurationBase(double t){fromSec(t);};
    ~DurationBase() {}
    T operator+(const T &rhs) const;
    T operator-(const T &rhs) const;
    T operator-() const;
    T operator*(double scale) const;
    T& operator+=(const T &rhs);
    T& operator-=(const T &rhs);
    T& operator*=(double scale);
    bool operator==(const T &rhs) const;
    inline bool operator!=(const T &rhs) const { return !(*static_cast<const T*>(this) == rhs); }
    bool operator>(const T &rhs) const;
    bool operator<(const T &rhs) const;
    bool operator>=(const T &rhs) const;
    bool operator<=(const T &rhs) const;
    double toSec() const { return (double)sec + 1e-9*(double)nsec; };
    int64_t toNSec() const {return (int64_t)sec*1000000000ll + (int64_t)nsec;  };
    T& fromSec(double t);
    T& fromNSec(int64_t t);
    bool isZero();
//    boost::posix_time::time_duration toBoost() const;
  };

  class UXVCOS_API Duration : public DurationBase<Duration>
  {
  public:
    Duration()
    : DurationBase<Duration>()
    { }

    Duration(int32_t _sec, int32_t _nsec)
    : DurationBase<Duration>(_sec, _nsec)
    {}

    explicit Duration(double t) { fromSec(t); }
  };

#endif // ROS_PACKAGE_NAME

typedef double Seconds;

static inline bool operator!(const Time& time) {
  return time.isZero();
}

} // namespace uxvcos

#endif // UXVCOS_TIME_H
