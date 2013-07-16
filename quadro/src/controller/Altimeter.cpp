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

#include "Altimeter.h"
#include "Quadrotor.h"

namespace uxvcos {
namespace Controller {

  Altimeter::Altimeter(RTT::TaskContext* parent, const std::string& name, const std::string& description)
    : Module(parent, name, description)
    , airborneHeight(0.20)
    , ultrasoundSwitchHeight(0.60)

    , ultrasoundOutlierElimination("UltrasoundOutlierElimination", 1.0)
    , ultrasoundFilter("UltrasoundFilter")
    , airborneFilter("AirborneFilter", 1.0)
    , controller(dynamic_cast<Quadrotor *>(parent))
  {
    this->addProperty("AirborneHeight", airborneHeight).doc("Ultrasound height above which the aircraft is considered as airborne");
    this->addProperty("UltrasoundSwitchHeight", ultrasoundSwitchHeight).doc("Ultrasound height above which the controller will switch to NAV mode automatically");

    this->addPort("sonar_height", portUltrasound).doc("Ultrasound measurements");
    this->addPort("height", portHeight).doc("Used height");

    controller->addPort(portUltrasound);
    controller->addPort(portHeight);

    this->addOperation("setMode", &Altimeter::setMode, this, RTT::OwnThread).doc("Set mode (AUTO = 1, ULTRASOUND = 2, NAV = 4, OFF = 128).").arg("value", "set value");
    this->addOperation("clearMode", &Altimeter::clearMode, this, RTT::OwnThread).doc("Clear mode (AUTO = 1, ULTRASOUND = 2, NAV = 4, OFF = 128).").arg("value", "clear value");
    this->addOperation("switchMode", &Altimeter::switchMode, this, RTT::OwnThread).doc("Switch mode (AUTO = 1, ULTRASOUND = 2, NAV = 4, OFF = 128).").arg("value", "new value");

    this->addProperty(ultrasoundFilter);
    this->addProperty(airborneFilter);
    this->addProperty(ultrasoundOutlierElimination);

    height.elevation = 0.0;
    height.height_above_ground = 0.0/0.0;
    switchMode(Height::AUTO);
  }

  Altimeter::~Altimeter() {}

  const Altimeter::Height& Altimeter::get() const {
    return height;
  }

  double Altimeter::getHeight() const {
    return height.height;
  }

  double Altimeter::getHeight(Mode mode) const {
    Height temp;
    getHeight(mode, temp);
    return temp.height;
  }

  void Altimeter::getHeight(Mode mode, Height& height) const {
    if (mode & Height::NAV) {
      height.height = controller->state.pose.pose.position.z - getElevation();
      height.vertical_speed = controller->velocity.z;
      height.header.stamp = controller->getTimestamp();
    } else if (mode & Height::ULTRASOUND) {
      height.height = ultrasoundFilter.y();
      height.vertical_speed = ultrasoundFilter.dy();
      height.header.stamp = controller->getTimestamp();
    } else  {
      height.height = 0.0;
      height.vertical_speed = 0.0;
      height.header.stamp = controller->getTimestamp();
    }
  }

  void Altimeter::execute() {
    // read Ultrasound height
    if (portUltrasound.read(ultrasound) == RTT::NewData && ultrasound.range > ultrasound.min_range && ultrasound.range < ultrasound.max_range - 0.1) {
      ultrasoundOutlierElimination.set(ultrasound.range, ultrasound.header.stamp.toSec());
      ultrasoundFilter.set(ultrasoundOutlierElimination.y(), controller->acceleration.z, ultrasoundOutlierElimination.t());
      airborneFilter.set(ultrasoundOutlierElimination.y(), ultrasoundOutlierElimination.t());

      // RTT::log(RTT::Debug) << "Ultrasound: t = " << ultrasoundFilter.t() << ", u = [" << ultrasoundOutlierElimination.y() << " " << controller->acceleration.z << "], y = [" << ultrasoundFilter.y() << "]" << RTT::endlog();

      // update elevation if difference is too high
      if ((getMode() & Height::ULTRASOUND) &&
          fabs((controller->state.pose.pose.position.z - height.elevation) - ultrasoundFilter.y()) > 0.5) {
        setElevation(controller->state.pose.pose.position.z - ultrasoundFilter.y());
      }

      height.height_above_ground = ultrasoundFilter.y();

      // set AIRBORNE state
      if (airborneFilter.y() > airborneHeight) {
        controller->setState(hector_uav_msgs::ControllerState::AIRBORNE);
      } else {
        controller->clearState(hector_uav_msgs::ControllerState::AIRBORNE);
      }

    }

    {
      double timeSinceLastUpdate = ultrasoundFilter.timeSinceLastUpdate(controller->getTimestamp().toSec());
      if (timeSinceLastUpdate > 0.5) {
        ultrasoundFilter.reset();
        height.height_above_ground = 0.0/0.0;
        if (getMode() & Height::ULTRASOUND) setMode(Height::AUTO);
      }
    }

    // set height control mode and update height
    updateMode();
    getHeight(getMode(), height);

    portHeight.write(height);
  }

  const Altimeter::Mode& Altimeter::updateMode() {
    return switchMode(getMode());
  }

  const Altimeter::Mode& Altimeter::setMode(Altimeter::Mode set) {
    return switchMode(getMode() | set);
  }

  const Altimeter::Mode& Altimeter::clearMode(Altimeter::Mode clear) {
    return switchMode(getMode() & ~clear);
  }

  const Altimeter::Mode& Altimeter::switchMode(Altimeter::Mode newMode) {
    const Mode& currentMode = getMode();

    do {
      if (newMode & Height::AUTO) {
        // switch between Ultrasound and Nav height automatically
        if (!ultrasoundFilter.isValid() || ultrasoundFilter.y() > ultrasoundSwitchHeight + 0.05) {
          newMode = (newMode | Height::NAV) & ~Height::ULTRASOUND;
        } else if (ultrasoundFilter.y() < ultrasoundSwitchHeight - 0.05) {
          newMode = (newMode | Height::ULTRASOUND) & ~Height::NAV;
        } else if (!(newMode & (Height::NAV | Height::ULTRASOUND))) {
          // default case when no mode (ULTRASOUND or NAV) is currently selected
          newMode = (newMode | Height::NAV);
        }
      }

      if (newMode & Height::ULTRASOUND) {
        newMode &= ~Height::NAV;
        if (!ultrasoundFilter.isValid()) return setMode(Height::AUTO);
        if (currentMode & Height::ULTRASOUND) break;

        // reset ultrasound filter
        ultrasoundFilter.reset(ultrasoundFilter.y());

        break;
      }

      if (newMode & Height::NAV) {
        newMode &= ~Height::ULTRASOUND;
        if (currentMode & Height::NAV) break;

//        if (currentMode & Height::ULTRASOUND && ultrasoundFilter.isValid()) {
//          // update elevation to prevent a jump in the control error
//          setElevation(controller->state.pose.pose.position.z - ultrasoundFilter.y());
//        }

        break;
      }

      return currentMode;
    } while(0);

    if (newMode != height.mode) RTT::log(RTT::Info) << "Altimeter switched to mode " << getModeString(newMode) << RTT::endlog();
    height.mode = newMode;
    return height.mode;
  }

  const Altimeter::Mode& Altimeter::getMode() const {
    return height.mode;
  }

  std::string Altimeter::getModeString() const {
    return getModeString(height.mode);
  }

  std::string Altimeter::getModeString(const Mode& mode) {
    std::string s;
    if (mode & Height::ULTRASOUND) s += "ULTRASOUND ";
    if (mode & Height::NAV) s += "NAV ";
    if (mode & Height::AUTO) s += "(AUTO) ";
    if (mode & Height::OFF) s += "(OFF) ";
    return s;
  }

  void Altimeter::setElevation(double value) {
    height.elevation = value;
    RTT::log(RTT::Info) << "New elevation (reference altitude) is " << height.elevation << " m" << RTT::endlog();
  }

  double Altimeter::getElevation() const {
    return height.elevation;
  }

  UltrasoundFilter::UltrasoundFilter(const std::string &name)
    : Filter::BaseFilter<double, boost::array<double,2> >(name)
    , l1(1.0), l2(1.0)
  {
    properties()->addProperty("L1", l1);
    properties()->addProperty("L2", l2);
    reset();
  }

  bool UltrasoundFilter::update(Input u) {
    if (!this->isValid()) {
      reset(u[0]);
      return true;
    }

    if (isnan(u[0])) return false;

    double error = u[0] - x[0];
    x[0] = x[0] + dt() * (x[1] + l1 * error);
    x[1] = x[1] + dt() * (u[1] + l2 * error);
    return true;
  }

  void UltrasoundFilter::reset(Output initial_value) {
    Filter::BaseFilter<double, boost::array<double,2> >::reset();
    x[0] = initial_value;
    x[1] = 0.0;
  }

  bool UltrasoundFilter::set(double height, double acceleration, double t) {
    Input u;
    u[0] = height;
    u[1] = acceleration;
    return set(u, t);
  }

} // namespace Controller
} // namespace uxvcos
