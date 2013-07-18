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

#include "IMU.h"

#include <rtt/os/TimeService.hpp>
#include <rtt/Logger.hpp>

namespace uxvcos {
namespace Sensors {

bool IMU::convert(const hector_uav_msgs::RawImu& raw) {
  double t = raw.header.stamp.toSec();

  data.linear_acceleration.x = Accel0.convert(raw.linear_acceleration[0]);
  data.linear_acceleration.y = Accel1.convert(raw.linear_acceleration[1]);
  data.linear_acceleration.z = Accel2.convert(raw.linear_acceleration[2]);
  mapAccel(&data.linear_acceleration.x);

  data.linear_acceleration.x = Filter_AccelX.set(data.linear_acceleration.x, t);
  data.linear_acceleration.y = Filter_AccelY.set(data.linear_acceleration.y, t);
  data.linear_acceleration.z = Filter_AccelZ.set(data.linear_acceleration.z, t);

  data.angular_velocity.x = Gyro0.convert(raw.angular_velocity[0]);
  data.angular_velocity.y = Gyro1.convert(raw.angular_velocity[1]);
  data.angular_velocity.z = Gyro2.convert(raw.angular_velocity[2]);
  mapGyro(&data.angular_velocity.x);

  data.angular_velocity.x = Filter_GyroX.set(data.angular_velocity.x, t);
  data.angular_velocity.y = Filter_GyroY.set(data.angular_velocity.y, t);
  data.angular_velocity.z = Filter_GyroZ.set(data.angular_velocity.z, t);

  if (setZeroInProgress) setZeroCommandUpdate(raw);
  return true;
}

bool IMU::setZeroCommandExecute(double seconds, unsigned int axes) {
  if (setZeroInProgress) return false;

  setZeroAxes = axes;
  if (setZeroAxes == 0) setZeroAxes = ~0;
  setZeroTimeout = RTT::os::TimeService::Instance()->getTicks() + RTT::os::TimeService::nsecs2ticks(RTT::Seconds_to_nsecs(seconds));
  setZeroBuffer.assign(6, 0.0);
  setZeroCount = 0;
  setZeroInProgress = true;
  return true;
}

// bool IMU::setZeroCommandCompleted(double seconds) {
//   return !setZeroInProgress && (setZeroCount > 0);
// }

void IMU::setZeroCommandUpdate(const hector_uav_msgs::RawImu& raw) {
  setZeroBuffer[0] = (setZeroBuffer[0] * setZeroCount + static_cast<double>(raw.linear_acceleration[0])) / (setZeroCount + 1);
  setZeroBuffer[1] = (setZeroBuffer[1] * setZeroCount + static_cast<double>(raw.linear_acceleration[1])) / (setZeroCount + 1);
  setZeroBuffer[2] = (setZeroBuffer[2] * setZeroCount + static_cast<double>(raw.linear_acceleration[2])) / (setZeroCount + 1);
  setZeroBuffer[3] = (setZeroBuffer[3] * setZeroCount + static_cast<double>(raw.angular_velocity[0]))  / (setZeroCount + 1);
  setZeroBuffer[4] = (setZeroBuffer[4] * setZeroCount + static_cast<double>(raw.angular_velocity[1]))  / (setZeroCount + 1);
  setZeroBuffer[5] = (setZeroBuffer[5] * setZeroCount + static_cast<double>(raw.angular_velocity[2]))  / (setZeroCount + 1);
  ++setZeroCount;

  if (RTT::os::TimeService::Instance()->getTicks() > setZeroTimeout) setZeroCommandFinish();
}

void IMU::setZeroCommandFinish() {
  if (setZeroCount > 0) {
    if ((setZeroAxes & (8u << mapAccel[0])) > 0) {
      Accel0.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[0] + .5));
      Accel0.offsetC.set(0);
      if (mapAccel[2] == 0) Accel0.offsetC.set(9.81);
    }

    if ((setZeroAxes & (8u << mapAccel[1])) > 0) {
      Accel1.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[1] + .5));
      Accel1.offsetC.set(0);
      if (mapAccel[2] == 1) Accel1.offsetC.set(9.81);
    }

    if ((setZeroAxes & (8u << mapAccel[2])) > 0) {
      Accel2.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[2] + .5));
      Accel2.offsetC.set(0);
      if (mapAccel[2] == 2) Accel2.offsetC.set(9.81);
    }

    if ((setZeroAxes & (1u << mapGyro[0])) > 0) {
      Gyro0.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[3] + .5));
      Gyro0.offsetC.set(0);
    }

    if ((setZeroAxes & (1u << mapGyro[1])) > 0) {
      Gyro1.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[4] + .5));
      Gyro1.offsetC.set(0);
    }

    if ((setZeroAxes & (1u << mapGyro[2])) > 0) {
      Gyro2.offsetA.set(-static_cast<unsigned short>(setZeroBuffer[5] + .5));
      Gyro2.offsetC.set(0);
    }

    RTT::log( RTT::Info ) << "Successfully calibrated IMU offsets using " << setZeroCount << " samples." << RTT::endlog();
  } else {
    RTT::log( RTT::Error ) << "No samples for IMU calibration!" << RTT::endlog();
  }

  setZeroInProgress = false;
}

} // namespace Sensors
} // namespace uxvcos
