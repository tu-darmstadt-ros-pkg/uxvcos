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

#include "EthernetAHRS.h"
#include "EthernetInterface.h"

namespace uxvcos {
namespace Interface {

EthernetAHRS::EthernetAHRS(EthernetInterface* interface, const std::string& name, const std::string& port_name, const std::string& description)
  : EthernetSensor<AHRS>(interface, name, port_name, description)
{}

EthernetAHRS::~EthernetAHRS()
{}

void EthernetAHRS::Set(const ArmAttitude_t& attitude) {
//  data.orientation.w = orientation.w();
//  data.orientation.x = orientation.x();
//  data.orientation.y = orientation.y();
//  data.orientation.z = orientation.z();
}

void EthernetAHRS::Set(const ArmBias_t& bias) {
}

void EthernetAHRS::Set(const ArmVelocity_t& velocity) {
//  data.v_n     = velocity.x;
//  data.v_e     = velocity.y;
//  data.v_d     = velocity.z;
}

void EthernetAHRS::addTo(EthernetSensorContainer &sensors) {
  sensors.insert(ARM_ATTITUDE_ID, sensor());
  sensors.insert(ARM_VELOCITY_ID, sensor());
  sensors.insert(ARM_BIAS_ID,     sensor());
}

void EthernetAHRS::request(Request &request) const {
  SET_REQUEST(ARM_ATTITUDE_ID, request);
  SET_REQUEST(ARM_VELOCITY_ID, request);
  // SET_REQUEST(ARM_BIAS_ID, request);
}

// this method will be overridden by the following one (this is an ugly workaround)
template<> bool EthernetSensor<AHRS>::decode(void *, size_t, MessageId) {
  return false;
}

bool EthernetAHRS::decode(void *payload, size_t length, MessageId id) {
  if (id == ARM_ATTITUDE_ID && length >= sizeof(ArmAttitude_t))
  {
    const struct ArmAttitude_t *ArmAttitude = reinterpret_cast<const struct ArmAttitude_t *>(payload);
    this->Set(*ArmAttitude);

    // if (!IS_REQUESTED(ARM_VELOCITY_ID, theRequest) && !IS_REQUESTED(ARM_BIAS_ID, theRequest)) this->updated(timestamp);
    this->updated(interface->getTimestamp());
    return true;
  }

  if (id == ARM_VELOCITY_ID && length >= sizeof(ArmVelocity_t))
  {
    const struct ArmVelocity_t *ArmVelocity = reinterpret_cast<const struct ArmVelocity_t *>(payload);
    this->Set(*ArmVelocity);

    // if (!IS_REQUESTED(ARM_ATTITUDE_ID, theRequest) && !IS_REQUESTED(ARM_BIAS_ID, theRequest)) ahrs->updated(timestamp);
    this->updated(interface->getTimestamp());
    return true;
  }

  if (id == ARM_BIAS_ID && length >= sizeof(ArmBias_t))
  {
    const struct ArmBias_t *ArmBias = reinterpret_cast<const struct ArmBias_t *>(payload);
    this->Set(*ArmBias);

    // if (!IS_REQUESTED(ARM_ATTITUDE_ID, theRequest) && !IS_REQUESTED(ARM_VELOCITY_ID, theRequest)) ahrs->updated(timestamp);
    this->updated(interface->getTimestamp());
    return true;
  }

  return false;
}

std::ostream& EthernetAHRS::operator >>(std::ostream& os) {
  char debugBuffer[256];
//  snprintf(debugBuffer, sizeof(debugBuffer) - 1, "Roll:%.3f  Pitch:%.3f  Yaw:%.3f  v_x:%.3f  v_y:%.3f  v_z:%.3f", // "AccelX:%.3f AccelY:%.3f AccelZ:%.3f GyroX:%.3f GyroY:%.3f GyroZ:%.3f",
//           data.rol*180.0/M_PI, data.pitch*180.0/M_PI, data.azimuth*180.0/M_PI,
//           data.v_n, data.v_e, data.v_d
//           // ArmBias->accelX, ArmBias->accelY, ArmBias->accelZ, ArmBias->gyroX*180.0/M_PI, ArmBias->gyroY*180.0/M_PI, ArmBias->gyroZ*180.0/M_PI
//          );
  return os << std::string(debugBuffer);
}

} // namespace Interface
} // namespace uxvcos
