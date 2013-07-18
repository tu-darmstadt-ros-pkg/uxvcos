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

#ifndef TESTBED_H
#define TESTBED_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>
#include <sensor_msgs/typekit/Imu.h>
#include <uxvcos/sensors/Sensor.h>

#include <uxvcos/Transformation.h>
#include <uxvcos/filter/PT1.h>

#include "hudaqlib/hudaqlib.h"

namespace uxvcos {

class Testbed : public RTT::TaskContext {
public:
  Testbed(const std::string& name = "Testbed");
  virtual ~Testbed();

  virtual bool setZeroCommandExecute(double seconds);

protected:
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

protected:
  virtual void setZeroCommandUpdate(double values[]);
  virtual void setZeroCommandFinish();

  Sensors::Sensor<sensor_msgs::Imu> reference;

  LinearTransformation<double,double> Encoder0, Encoder1, Encoder2;
  Filter::PT1 Filter0, Filter1, Filter2;

  bool setZeroInProgress;
  RTT::os::TimeService::ticks setZeroTimeout;
  unsigned int setZeroCount;
  double setZeroBuffer[3];

private:
  HUDAQHANDLE h;
};

} // namespace uxvcos

#endif // TESTBED_H

