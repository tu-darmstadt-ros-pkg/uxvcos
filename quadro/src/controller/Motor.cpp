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

#include "Motor.h"
#include "Quadrotor.h"

#include <uxvcos/controller/Limit.h>
#include "wrench_operations.h"

namespace uxvcos {
namespace Controller {

  Motor::Motor(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, hector_uav_msgs::MotorPWM, hector_uav_msgs::MotorCommand>(controller, name, description)
    , eventOverload("Overload")
    , usePropulsionModel("UsePropulsionModel", "Use motor voltages from propulsion model", false)
    , on(false)
    , load(0.0)
  {
    this->addPort(eventOverload).doc("Overload event");
    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addPort(portOutput);
    controller->addPort(eventOverload);

    this->addAttribute("on", on);
    this->addAttribute("load", load);
    this->addOperation("engage", &Motor::engage, this);
    this->addOperation("shutdown", &Motor::shutdown, this);

    this->addProperty(usePropulsionModel);

    output.pwm.resize(4);
  }

  bool Motor::update(double dt) {
    const Limit<double> limit(20.0,255.0);
    double pwm[4] = { 0.0, 0.0, 0.0, 0.0 };

    if (usePropulsionModel.get()) {
      for (int i = 0; i < 4; ++i) {
        pwm[i] = input.voltage[i] / 14.8 * 255.0;
      }

    } else {
      geometry_msgs::WrenchStamped forces = controller->getWrench();
      forces.wrench.force.z  /= 4 * PWM2N;
      forces.wrench.torque.x /= 2 * l_m * PWM2N;
      forces.wrench.torque.y /= 2 * l_m * PWM2N;
      forces.wrench.torque.z /= 4 * PWM2NM;

      pwm[0] = forces.wrench.force.z - forces.wrench.torque.y + forces.wrench.torque.z;
      pwm[1] = forces.wrench.force.z - forces.wrench.torque.x - forces.wrench.torque.z;
      pwm[2] = forces.wrench.force.z + forces.wrench.torque.y + forces.wrench.torque.z;
      pwm[3] = forces.wrench.force.z + forces.wrench.torque.x - forces.wrench.torque.z;
    }

    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::min();
    double overload = 0;

    for(int i = 0; i < 4; i++) {
      // if (pwm[i] < limit.Min()) pwm[i] = limit.Min();
      if (max < pwm[i]) max = pwm[i];
      if (min > pwm[i]) min = pwm[i];
    }
    load = max / limit.Max();

    if (max > limit.Max()) {
      overload = max - limit.Max() + std::numeric_limits<double>::epsilon();
      eventOverload.write(load);
    } else if (min < limit.Min()) {
      overload = -(limit.Min() - min) - std::numeric_limits<double>::epsilon();
    }

    output.pwm[0] = static_cast<unsigned char>(limit(pwm[0] - overload));
    output.pwm[1] = static_cast<unsigned char>(limit(pwm[1] - overload));
    output.pwm[2] = static_cast<unsigned char>(limit(pwm[2] - overload));
    output.pwm[3] = static_cast<unsigned char>(limit(pwm[3] - overload));

    if (on) {
      if (controller->motor_status.frequency.size() >= 4) {
        if (controller->motor_status.frequency[0] > 50.0 &&
            controller->motor_status.frequency[1] > 50.0 &&
            controller->motor_status.frequency[2] > 50.0 &&
            controller->motor_status.frequency[3] > 50.0) {
          controller->setState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
        } else if (controller->motor_status.frequency[0] < 50.0 &&
                   controller->motor_status.frequency[1] < 50.0 &&
                   controller->motor_status.frequency[2] < 50.0 &&
                   controller->motor_status.frequency[3] < 50.0) {
          //controller->clearState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
        }
      }
    } else {
      output.pwm[0] = 0;
      output.pwm[1] = 0;
      output.pwm[2] = 0;
      output.pwm[3] = 0;
      controller->clearState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
    }

    return true;
  }

  void Motor::afterStop() {
    shutdown();
  }

  void Motor::engage() {
    if (on == true) return;
    RTT::log(RTT::Info) << "Switching motors on!" << RTT::endlog();
    on = true;
  }

  void Motor::shutdown() {
    if (on == false) return;
    RTT::log(RTT::Info) << "Switching motors off!" << RTT::endlog();
    output.pwm[0] = 0;
    output.pwm[1] = 0;
    output.pwm[2] = 0;
    output.pwm[3] = 0;
    setOutput(output);
    controller->clearState(hector_uav_msgs::ControllerState::MOTORS_RUNNING);
    on = false;
  }

} // namespace Controller
} // namespace uxvcos
