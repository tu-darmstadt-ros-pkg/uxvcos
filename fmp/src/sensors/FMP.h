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

#ifndef UXVCOS_SENSORS_FMP_H
#define UXVCOS_SENSORS_FMP_H

#include <fmp_msgs/typekit/Types.h>
#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>

#include <limits>

namespace uxvcos {
namespace Sensors {

class FMP : public RawSensor<fmp_msgs::RawData, fmp_msgs::SensorData> {
public:
  FMP(RTT::TaskContext* parent, const std::string& name = "FMP", const std::string& port_name = "fmp", const std::string& description = "FMP sensor data")
    : RawSensor<fmp_msgs::RawData, fmp_msgs::SensorData>(parent, name, port_name, description)
    , voltage("Voltage")
    , temp("Temperature")
    , alpha("alpha")
    , beta("beta")
    , vTAS("TAS")
    , pStat("pStat")
    , pDyn("pDyn")
    , aileron("Aileron")
    , elevator("Elevator")
    , rudder("Rudder")
    , elevatorTrim("ElevatorTrim")
    , wMot("Motor")
    , humi("Humidity")
  {
    properties()->addProperty(voltage);
    properties()->addProperty(temp);
    properties()->addProperty(humi);
    properties()->addProperty(alpha);
    properties()->addProperty(beta);
    properties()->addProperty(vTAS);
    properties()->addProperty(pStat);
    properties()->addProperty(pDyn);
    properties()->addProperty(aileron);
    properties()->addProperty(elevator);
    properties()->addProperty(rudder);
    properties()->addProperty(elevatorTrim);
    properties()->addProperty(wMot);
  }

  virtual bool convert(const fmp_msgs::RawData& raw) {
    static const double nan = std::numeric_limits<double>::quiet_NaN();

    data.voltage      = raw.voltage != 0xFFFF      ? voltage.convert(raw.voltage)           : nan;
    data.temp         = raw.temp != 0xFFFF         ? temp.convert(raw.temp)                 : nan;
    data.humi         = raw.humi != 0xFFFF         ? humi.convert(raw.humi)                 : nan;
    data.alpha        = raw.alpha != 0xFFFF        ? alpha.convert(raw.alpha)               : nan;
    data.beta         = raw.beta != 0xFFFF         ? beta.convert(raw.beta)                 : nan;
    data.vTAS         = raw.vTAS != 0xFFFF         ? vTAS.convert(raw.vTAS)                 : nan;
    data.pStat        = raw.pStat != 0xFFFF        ? pStat.convert(raw.pStat)               : nan;
    data.pDyn         = raw.pDyn != 0xFFFF         ? pDyn.convert(raw.pDyn)                 : nan;
    data.aileron      = raw.aileron != 0xFFFF      ? aileron.convert(raw.aileron)           : nan;
    data.elevator     = raw.elevator != 0xFFFF     ? elevator.convert(raw.elevator)         : nan;
    data.rudder       = raw.rudder != 0xFFFF       ? rudder.convert(raw.rudder)             : nan;
    data.elevatorTrim = raw.elevatorTrim != 0xFFFF ? elevatorTrim.convert(raw.elevatorTrim) : nan;
    data.wMot         = raw.wMot != 0xFFFF         ? wMot.convert(raw.wMot)                 : nan;
    updated();
    return true;
  }

protected:
  LinearTransformation<int,double> voltage, temp, alpha, beta, vTAS, pStat, pDyn, aileron, elevator, rudder, elevatorTrim, wMot;
  QuadraticTransformation<int,double> humi;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSORS_FMP_H
