//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#include "Test.h"
#include "Quadrotor.h"

#include <uxvcos/controller/Limit.h>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

namespace uxvcos {
namespace Controller {

  Test::Test(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, hector_uav_msgs::MotorPWM>(controller, name, description)
    , mean("Mean", 0.0)
    , amplitude("Amplitude", 0.0)
    , period("Period", 0.0)
    , sigma("Sigma", 0.0)
  {
    this->addOperation("setPWM", &Test::setPWM, this, RTT::OwnThread).doc("Set PWM directly").arg("front", "").arg("right", "").arg("rear", "").arg("left", "");

    this->addAttribute(mean);
    this->addAttribute(amplitude);
    this->addAttribute(period);
    this->addAttribute(sigma);

    this->addPort("test_input", test_input);
    controller->addPort(test_input);

    output.pwm.resize(4);
  }

  bool Test::update(double dt) {
    const Limit<double> limit(20.0,255.0);
    double t = (uxvcos::Time::now() - startOffset).toSec();

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, boost::normal_distribution<>(0.0, sigma.get()));

    if (test_input.read(this->output) == RTT::NewData) {
      mean.set(-1.0);
    }

    if (mean.get() > 0.0) {
      double pwm = mean.get() + amplitude.get() * (period.get() > 0.0 ? -cos(2.0 * M_PI * t/period.get()) : 0.0);
      //RTT::log(RTT::Debug) << "setting motors to " << static_cast<unsigned int>(pwm) << " PWM" << RTT::endlog();

      output.pwm[0] = static_cast<unsigned char>(limit(pwm + var_nor()));
      output.pwm[1] = static_cast<unsigned char>(limit(pwm + var_nor()));
      output.pwm[2] = static_cast<unsigned char>(limit(pwm + var_nor()));
      output.pwm[3] = static_cast<unsigned char>(limit(pwm + var_nor()));

      controller->setState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);

    } else if (mean.get() == 0) {
      output.pwm[0] = 0;
      output.pwm[1] = 0;
      output.pwm[2] = 0;
      output.pwm[3] = 0;
      controller->clearState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
    }

    return true;
  }

  bool Test::beforeStart() {
    if (controller->getController("Motor")->isActive()) return false;
    return true;
  }

  void Test::reset() {
    startOffset = uxvcos::Time::now();
  }

  void Test::setOutput(const hector_uav_msgs::MotorPWM &new_value)
  {
    Controller<Quadrotor, hector_uav_msgs::MotorPWM>::setOutput(new_value);
    controller->writeOutput(getOutput());
  }

  void Test::setPWM(int pwm0, int pwm1, int pwm2, int pwm3) {
    const Limit<int> limit(0, 255);

    output.pwm[0] = static_cast<unsigned char>(limit(pwm0));
    output.pwm[1] = static_cast<unsigned char>(limit(pwm1));
    output.pwm[2] = static_cast<unsigned char>(limit(pwm2));
    output.pwm[3] = static_cast<unsigned char>(limit(pwm3));
    mean.set(0.0/0.0); // disable mean/amplitude/period/sigma test mode 

    setOutput(output);
  }

  void Test::afterStop() {
    output.pwm[0] = 0;
    output.pwm[1] = 0;
    output.pwm[2] = 0;
    output.pwm[3] = 0;

    setOutput(output);
    controller->clearState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
  }

} // namespace Controller
} // namespace uxvcos
