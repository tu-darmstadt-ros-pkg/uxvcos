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

#include "EthernetInterface.h"
#include "EthernetSensor.h"
#include "EthernetIMU.h"
#include "EthernetAHRS.h"

#include <uxvcos/interface/RC.h>

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/sensors/Supply.h>
#include <uxvcos/sensors/Magnetic.h>
#include <uxvcos/sensors/Ultrasound.h>
#include <uxvcos/sensors/Baro.h>
#include <uxvcos/sensors/Temperature.h>
#include <uxvcos/sensors/Airspeed.h>

#include <hector_std_msgs/UInt16Array.h>
#include <hector_std_msgs/UInt16.h>
#include <hector_uav_msgs/MotorStatus.h>

using namespace RTT;

namespace uxvcos {
namespace Interface {

void EthernetInterface::setupSensors() {
  addSensor(new EthernetSensor<Sensors::Supply>(this, ARM_INTERNAL_AD_ID));
  addSensor(new EthernetIMU(this));
  addSensor(new EthernetAHRS(this));
  addSensor(new EthernetSensor<Sensors::Magnetic>(this, ARM_RAW_MAG3D_ID));
  addSensor(new EthernetSensor<Sensors::Ultrasound>(this, ARM_US_SENSOR_ID));
  addSensor(new EthernetSensor<Sensors::Baro>(this, ARM_RAW_BARO_ID));
  addSensor(new EthernetSensor<Sensors::Temperature>(this, ARM_RAW_BARO_TEMP_ID));
  addSensor(new EthernetSensor<Sensors::Airspeed>(this, ARM_RAW_AIRSPEED_ID));
  addSensor(new EthernetSensor<RC>(this, ARM_RC_ID));
  addSensor(new EthernetSensor<Sensors::Sensor<hector_uav_msgs::MotorStatus> >(this, "MotorStatus", "motor_status", "Frequency and current information", ARM_MOTOR_ID));
  addSensor(new EthernetSensor<Sensors::Sensor<hector_std_msgs::UInt16Array> >(this, "Analog0", "analog0", "Onboard A/D converter", ARM_RAW_AD0_ID, 0x80 | 0x0A));
  addSensor(new EthernetSensor<Sensors::Sensor<hector_std_msgs::UInt16Array> >(this, "Analog1", "analog1", "External A/D converter", ARM_RAW_AD1_ID, 0x80 | 0x0B));
  addSensor(new EthernetSensor<Sensors::Sensor<hector_std_msgs::UInt16Array> >(this, "Analog2", "analog2", "ARM internal A/D converter", ARM_INTERNAL_AD_ID, 0x80 | 0x0C));
}

template <>
bool EthernetSensor<Sensors::Sensor<hector_std_msgs::UInt16Array> >::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmAD16_t)) return false;
  struct ArmAD16_t *ArmAD16 = reinterpret_cast<struct ArmAD16_t *>(payload);

  static const std::size_t size = sizeof(ArmAD16_t::value) / sizeof(ArmAD16_t::value[0]);
  data.data.resize(size);
  for(std::size_t i = 0; i < size; ++i) data.data[i] = ArmAD16->value[i];

  // if (key) data.setKey(key);
  data.header.stamp = interface->getTimestamp();
  updated(interface->getTimestamp());

  return true;
}

template <>
bool EthernetSensor<Sensors::Supply>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmInternalAD16_t)) return false;
  struct ArmInternalAD16_t *ArmAD16 = reinterpret_cast<struct ArmInternalAD16_t *>(payload);

  data.voltage.resize(1);
  data.current.resize(2);
  data.voltage[0] = static_cast<double>(ArmAD16->value[0]) * 0.020913565;
  data.current[0] = static_cast<double>(ArmAD16->value[1]) * 17.233 * 1e-3;
  data.current[1] = static_cast<double>(ArmAD16->value[2]) * 17.233 * 1e-3;

  updated(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Magnetic>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmRawMag3D_t)) return false;
  struct ArmRawMag3D_t *ArmMag3D = reinterpret_cast<struct ArmRawMag3D_t *>(payload);

  raw.channel[0] = ArmMag3D->value[0];
  raw.channel[1] = ArmMag3D->value[1];
  raw.channel[2] = ArmMag3D->value[2];

  if (raw.channel[0] != 0.0 && raw.channel[1] != 0.0 && raw.channel[2] != 0.0) {
    updatedRaw(interface->getTimestamp());
  } else {
    RTT::log(RTT::Warning) << "Received illegal response from magnetometer: [" << raw.channel[0] << "," << raw.channel[1] << "," << raw.channel[2] << "]" << RTT::endlog();
    return false;
  }
  return true;
}

template <>
bool EthernetSensor<RC>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmRC_t)) return false;
  struct ArmRC_t *ArmRC = reinterpret_cast<struct ArmRC_t *>(payload);

  raw.status = ArmRC->status;
  raw.channel.resize(ARMRC_CHANNELS);
  std::size_t i = 0;
  for( ; i < ARMRC_CHANNELS && i < raw.channel.size(); ++i) raw.channel[i] = ArmRC->value[i];

  // set channels not included in the message to 0
  for( ; i < raw.channel.size(); ++i) raw.channel[i] = 0;

  updatedRaw(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Ultrasound>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmUS_t)) return false;
  struct ArmUS_t *ArmUS = reinterpret_cast<struct ArmUS_t *>(payload);

  raw.data.resize(ARMUS_SENSORS);
  for(unsigned int i = 0; i < ARMUS_SENSORS; ++i) raw.data[i] = ArmUS->value[i];

  updatedRaw(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Baro>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmRawBaro_t)) return false;
  struct ArmRawBaro_t *ArmBaro = reinterpret_cast<struct ArmRawBaro_t *>(payload);

  raw.data = ArmBaro->value;

  updatedRaw(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Temperature>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmRawBaroTemp_t)) return false;
  struct ArmRawBaroTemp_t *ArmTemperature = reinterpret_cast<struct ArmRawBaroTemp_t *>(payload);

  raw.data = ArmTemperature->value;

  updatedRaw(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Airspeed>::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmRawBaro_t)) return false;
  struct ArmRawBaro_t *ArmAirspeed = reinterpret_cast<struct ArmRawBaro_t *>(payload);

  raw.data = ArmAirspeed->value;

  updatedRaw(interface->getTimestamp());
  return true;
}

template <>
bool EthernetSensor<Sensors::Sensor<hector_uav_msgs::MotorStatus> >::decode(void *payload, size_t length, MessageId) {
  if (length < sizeof(ArmMotor_t)) return false;
  struct ArmMotor_t *ArmMotor = reinterpret_cast<struct ArmMotor_t *>(payload);

  data.on = ArmMotor->enabled;
  data.running = false;
  data.voltage.resize(ARMMOTOR_COUNT);
  data.frequency.resize(ARMMOTOR_COUNT);
  data.current.resize(ARMMOTOR_COUNT);

  boost::shared_ptr<Motor> motor = interface->getModule<Motor>("Motor");
  boost::shared_ptr<EthernetSensor<Sensors::Supply> > supply = interface->getModule<EthernetSensor<Sensors::Supply> >("Supply");
  if (motor && supply && supply->isEnabled() && supply->get().voltage.size() > 0) {
    data.voltage.resize(std::min(std::size_t(ARMMOTOR_COUNT), motor->getMotorCommand().pwm.size()));
    for(std::size_t i = 0; i < data.voltage.size(); ++i) {
      data.voltage[i] = motor->getMotorCommand().pwm[i] * supply->get().voltage[0] / 255.0;
    }
  } else {
    data.voltage.assign(ARMMOTOR_COUNT, 0.0);
  }

  for(std::size_t i = 0; i < ARMMOTOR_COUNT; ++i) {
    data.frequency[i] = ArmMotor->Frequency[i];
    data.current[i] = ArmMotor->Current[i];
    if (data.frequency[i] > 0.0) data.running = true;
  }

  updated(interface->getTimestamp());
  return true;
}

} // namespace Interface
} // namespace uxvcos
