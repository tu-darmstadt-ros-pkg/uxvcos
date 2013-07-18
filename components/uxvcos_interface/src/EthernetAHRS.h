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

#ifndef INTERFACE_ETHERNETAHRS_H
#define INTERFACE_ETHERNETAHRS_H

#include <uxvcos/sensors/Sensor.h>
#include "EthernetSensor.h"
#include <sensor_msgs/typekit/Imu.h>

namespace uxvcos {
namespace Interface {

typedef Sensors::Sensor<sensor_msgs::Imu> AHRS;

class EthernetAHRS : public EthernetSensor<AHRS>
{
public:
    EthernetAHRS(EthernetInterface* interface, const std::string& name = "AHRS", const std::string& port_name = "ahrs_imu", const std::string& description = "Attitude and Heading Reference System");
    virtual ~EthernetAHRS();

    virtual void Set(const ArmAttitude_t& attitude);
    virtual void Set(const ArmBias_t& bias);
    virtual void Set(const ArmVelocity_t& velocity);

    virtual void addTo(EthernetSensorContainer &sensors);
    virtual void request(Request &request) const;

    virtual bool decode(void *payload, size_t length, MessageId id);
    virtual std::ostream& operator>>(std::ostream& os);
};

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_ETHERNETAHRS_H
