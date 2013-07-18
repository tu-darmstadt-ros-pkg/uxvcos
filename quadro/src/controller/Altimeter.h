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

#ifndef CONTROLLER_ALTIMETER_H
#define CONTROLLER_ALTIMETER_H

#include <uxvcos/Module.h>
#include <rtt/Port.hpp>
#include <sensor_msgs/typekit/Range.h>
#include <quadro_msgs/typekit/Height.h>

#include <uxvcos/filter/OutlierElimination.h>
#include <uxvcos/filter/PT1.h>

#include <boost/array.hpp>

namespace uxvcos {
namespace Controller {

class UltrasoundFilter : public Filter::BaseFilter<double, boost::array<double,2> >
{
public:
  using Filter::BaseFilter<double, boost::array<double,2> >::Output;
  using Filter::BaseFilter<double, boost::array<double,2> >::Input;
  typedef boost::array<double,2> State;

  UltrasoundFilter(const std::string& name);
  void reset(Output initial_value = Filter::nan<Output>());
  double y() const { return x[0]; }
  double dy() const { return x[1]; }
  bool update(Input u);

  using Filter::BaseFilter<double, boost::array<double,2> >::set;
  bool set(double height, double acceleration, double t);

public:
  State x;
  double l1, l2;
};

class Quadrotor;

class Altimeter : public Module
{
public:
  typedef quadro_msgs::Height Height;
  typedef Height::_mode_type Mode;

  Altimeter(RTT::TaskContext* controller, const std::string& name = "Altimeter", const std::string& description = "");
   ~Altimeter();

  const Height& get() const;

  double getHeight() const;
  double getHeight(Mode mode) const;
  void getHeight(Mode mode, Height&) const;

  void setElevation(double value);
  double getElevation() const;

  const Mode& getMode() const;
  const Mode& setMode(Mode set);
  const Mode& clearMode(Mode clear);
  const Mode& switchMode(Mode newMode);

  std::string getModeString() const;
  static std::string getModeString(const Mode& mode);

protected:
  void execute();
  const Mode& updateMode();

private:
  RTT::InputPort<sensor_msgs::Range> portUltrasound;
  sensor_msgs::Range ultrasound;

  RTT::OutputPort<quadro_msgs::Height> portHeight;
  Height height;

  double airborneHeight;
  double ultrasoundSwitchHeight;

  Filter::OutlierElimination<double> ultrasoundOutlierElimination;
  UltrasoundFilter ultrasoundFilter;
  Filter::PT1 airborneFilter;

  Quadrotor *controller;
};

} // namespace Controller
} // namespace uxvcos

#endif // CONTROLLER_ALTIMETER_H
