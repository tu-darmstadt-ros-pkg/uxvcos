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

#ifndef INTERFACE_RC_H
#define INTERFACE_RC_H

#include <uxvcos/sensors/Sensor.h>
#include <uxvcos/Transformation.h>
#include <uxvcos/Configuration.h>

#include <hector_uav_msgs/typekit/RawRC.h>
#include <hector_uav_msgs/typekit/RC.h>

#include <vector>

namespace uxvcos {
namespace Interface {

class RC : public Sensors::RawSensor<hector_uav_msgs::RawRC,hector_uav_msgs::RC> {
public:
  RC(RTT::TaskContext* parent, const std::string& name = "RC", const std::string& port_name = "rc", const std::string& description = "Radio Control Receiver");

  bool convert(const hector_uav_msgs::RawRC &update);

protected:
  bool initialize();
  void cleanup();

protected:
  Configuration configuration;
  class Axis;
  class Switch;
  std::vector<boost::shared_ptr<Axis> > axes;
  std::vector<boost::shared_ptr<Switch> > switches;
};

std::ostream& operator<<(std::ostream& os, const RC& rc);

} // namespace Interface
} // namespace uxvcos

#endif // INTERFACE_RC_H
