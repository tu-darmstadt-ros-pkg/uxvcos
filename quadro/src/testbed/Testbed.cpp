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

#include "Testbed.h"
#include <rtt/os/TimeService.hpp>
#include <rtt/Component.hpp>

#include <math.h>

using namespace RTT;

namespace uxvcos {

Testbed::Testbed(const std::string& name)
  : RTT::TaskContext(name)
  , reference(this, "ReferenceAttitude", "reference_orientation", "Reference orientation from the quadrotor testbed")
  , Encoder0("Encoder0"), Encoder1("Encoder1"), Encoder2("Encoder2")
  , Filter0("Filter0", 0.0), Filter1("Filter1", 0.0), Filter2("Filter2", 0.0)
  , setZeroInProgress(false)
{
  this->addProperty(Encoder0);
  this->addProperty(Encoder1);
  this->addProperty(Encoder2);
  this->addProperty(Filter0);
  this->addProperty(Filter1);
  this->addProperty(Filter2);

  this->provides()->addOperation("setZero", &Testbed::setZeroCommandExecute, this, RTT::OwnThread).doc("Remember the current Testbed measurements as zero").arg("seconds", "Integration time");
}

Testbed::~Testbed() {
  stop();
  cleanup();
}

bool Testbed::startHook() {
  /* open a handle to the first MF624 device in the system */
  h = HudaqOpenDevice("MF624", 1, 0);
  if (h == 0)
  {
    log(Error) << "Data acquisition device not found!" << endlog();
    return false;
  }

  setZeroInProgress = false;
  Filter0.reset();
  Filter1.reset();
  Filter2.reset();

  return true;
}

void Testbed::updateHook() {

  /* Buffer for channel numbers. Order of channels is not significant.
     Duplicated channels are also supported. */
  unsigned channels[3] = {0,2,1};
  /* base::Buffer for receiving values read. Is size must correspond to
     buffer of channels. */
  double values[3];

  // Data::Timestamp::ticks sendTimestamp = RTT::os::TimeService::Instance()->getTicks();

  /* read all the analog inputs together */
  if (HudaqAIReadMultiple(h,3,channels,values) != HUDAQSUCCESS) {
    log(Error) << "Data acquisition failed!" << endlog();
    return;
  }

  // double time = RTT::os::TimeService::Instance()->secondsSince(sendTimestamp);
  // printf("ADC took %f ms.", time * 1000.0);

  if (setZeroInProgress) setZeroCommandUpdate(values);

  Time now = ros::Time::now();
  double dt = (now - reference.getTimestamp()).toSec();
  double a = Filter0.operator()(Encoder0.convert(values[0]), dt);
  double b = Filter1.operator()(Encoder1.convert(values[1]), dt);
  double c = Filter2.operator()(Encoder2.convert(values[2]), dt);

  // attitude.rol     = a;
  // attitude.pitch   = b;
  // attitude.azimuth = c;

  // R = R_3(c) * R_2(b) * R_1(a)

  double cang[] = { cos(a/2), cos(b/2), cos(c/2) };
  double sang[] = { sin(a/2), sin(b/2), sin(c/2) };

//  case 'xyz'
//      q = [ cang(:,1).*cang(:,2).*cang(:,3) - sang(:,1).*sang(:,2).*sang(:,3), ...
//          cang(:,1).*sang(:,2).*sang(:,3) + sang(:,1).*cang(:,2).*cang(:,3), ...
//          cang(:,1).*sang(:,2).*cang(:,3) - sang(:,1).*cang(:,2).*sang(:,3), ...
//          cang(:,1).*cang(:,2).*sang(:,3) + sang(:,1).*sang(:,2).*cang(:,3)];

  geometry_msgs::Quaternion q0  = reference.data.orientation;
  geometry_msgs::Quaternion &q1 = reference.data.orientation;
  q1.w = cang[0]*cang[1]*cang[2] - sang[0]*sang[1]*sang[2];
  q1.x = cang[0]*sang[1]*sang[2] + sang[0]*cang[1]*cang[2];
  q1.y = cang[0]*sang[1]*cang[2] - sang[0]*cang[1]*sang[2];
  q1.z = cang[0]*cang[1]*sang[2] + sang[0]*sang[1]*cang[2];

//  double r11 =  sin(a)*sin(b)*cos(c) + cos(a)*sin(c);
//  double r12 =         cos(b)*cos(c);
//  double r21 =  cos(a)*sin(b)*cos(c) - sin(a)*sin(c);
//  double r31 =  cos(a)*sin(b)*sin(c) + sin(a)*cos(c);
//  double r32 =  cos(a)*cos(b);

//  attitude.rol     = atan2( r31, r32 );
//  attitude.pitch   = asin( r21 );
//  attitude.azimuth = atan2( r11, r12 );

  geometry_msgs::Quaternion qd;
  qd.w = (q1.w - q0.w) / dt;
  qd.x = (q1.x - q0.x) / dt;
  qd.y = (q1.y - q0.y) / dt;
  qd.z = (q1.z - q0.z) / dt;

  reference.data.angular_velocity.x = 2 * (-q1.x * qd.w + q1.w * qd.x + q1.z * qd.y - q1.y * qd.z);
  reference.data.angular_velocity.y = 2 * (-q1.y * qd.w - q1.z * qd.x + q1.w * qd.y + q1.x * qd.z);
  reference.data.angular_velocity.z = 2 * (-q1.z * qd.w + q1.y * qd.x - q1.x * qd.y + q1.w * qd.z);

  reference.updated(now);
}

void Testbed::stopHook() {
  /* close the device handle */
  HudaqCloseDevice(h);
}

bool Testbed::setZeroCommandExecute(double seconds) {
  if (setZeroInProgress) return false;

  setZeroTimeout = RTT::os::TimeService::Instance()->getTicks() + RTT::os::TimeService::nsecs2ticks(RTT::Seconds_to_nsecs(seconds));
  setZeroBuffer[0] = 0;
  setZeroBuffer[1] = 0;
  setZeroBuffer[2] = 0;
  setZeroCount = 0;
  setZeroInProgress = true;
  return true;
}

void Testbed::setZeroCommandUpdate(double values[]) {
  setZeroBuffer[0] = (setZeroBuffer[0] * setZeroCount + values[0]) / (setZeroCount + 1);
  setZeroBuffer[1] = (setZeroBuffer[1] * setZeroCount + values[1]) / (setZeroCount + 1);
  setZeroBuffer[2] = (setZeroBuffer[2] * setZeroCount + values[2]) / (setZeroCount + 1);
  ++setZeroCount;

  if (RTT::os::TimeService::Instance()->getTicks() > setZeroTimeout) setZeroCommandFinish();
}

void Testbed::setZeroCommandFinish() {
  if (setZeroCount > 0) {
    Encoder0.offsetA.set(-setZeroBuffer[0]);
    Encoder1.offsetA.set(-setZeroBuffer[1]);
    Encoder2.offsetA.set(-setZeroBuffer[2]);

    RTT::log( RTT::Info ) << "Successfully calibrated Testbed offsets." << RTT::endlog();
  }

  setZeroInProgress = false;
}

ORO_CREATE_COMPONENT( Testbed )

} // namespace uxvcos
