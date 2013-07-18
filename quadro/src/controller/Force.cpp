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

#include "Force.h"
#include "Quadrotor.h"
#include "wrench_operations.h"

namespace uxvcos {
namespace Controller {

  Force::Force(Quadrotor* controller, const std::string& name, const std::string& description)
    : Controller<Quadrotor, hector_uav_msgs::MotorCommand, geometry_msgs::WrenchStamped>(controller, name, description)
  {
    controller->addPort(portCommand);
    controller->addPort(portInput);
    controller->addPort(portOutput);

    output.force.resize(4);
    output.frequency.resize(4);
    output.torque.resize(4);
    output.voltage.resize(4);
  }

  bool Force::update(double dt) {
    input = controller->getWrench();
    input.header.stamp = controller->getTimestamp();

    // CT = CT3*J3 + CT2*J2 + CT1*J + CT0
    static const double CT3 = -0.0350;
    static const double CT2 =  0.0081;
    static const double CT1 = -0.0100;
    static const double CT0 =  0.0133;

    // CM = CM3*J3 + CM2*J2 + CM1*J + CM0
    static const double CM3 = -7.2696e-004;
    static const double CM2 = -6.6376e-005;
    static const double CM1 =  8.0233e-005;
    static const double CM0 =  1.8968e-004;

    static const double R_P =  0.1269;
    // Motordaten aus Roxxy2827-34 Identi mit 10x4.5 Schraube
    static const double Psi =  0.011099903912915;
    static const double R_A =  0.506251525173286;

    static const double rho = 1.225;

    double sin_rol = sin(controller->euler.roll);
    double sin_pit = sin(controller->euler.pitch);
    double sin_yaw = sin(controller->euler.yaw);
    double cos_rol = cos(controller->euler.roll);
    double cos_pit = cos(controller->euler.pitch);
    double cos_yaw = cos(controller->euler.yaw);
    double v_z = (cos_rol*sin_pit*cos_yaw+sin_rol*sin_yaw)*controller->state.twist.twist.linear.x + (cos_rol*sin_pit*sin_yaw-sin_rol*cos_yaw)*controller->state.twist.twist.linear.y + (cos_rol*cos_pit)*controller->state.twist.twist.linear.z;

    double v1[4];
    v1[0] = v_z + controller->imu.angular_velocity.y * l_m;
    v1[1] = v_z + controller->imu.angular_velocity.x * l_m;
    v1[2] = v_z - controller->imu.angular_velocity.y * l_m;
    v1[3] = v_z - controller->imu.angular_velocity.x * l_m;

    double J[4] = { 0.0, 0.0, 0.0, 0.0 };
    if (controller->motor_status.frequency.size() >= 4) {
      if (controller->motor_status.frequency[0] > 0.0) J[0] = v1[0] * M_PI / controller->motor_status.frequency[0] / R_P;
      if (controller->motor_status.frequency[1] > 0.0) J[1] = v1[1] * M_PI / controller->motor_status.frequency[1] / R_P;
      if (controller->motor_status.frequency[2] > 0.0) J[2] = v1[2] * M_PI / controller->motor_status.frequency[2] / R_P;
      if (controller->motor_status.frequency[3] > 0.0) J[3] = v1[3] * M_PI / controller->motor_status.frequency[3] / R_P;
    }

    double C[4];
    for(int i = 0; i < 4; ++i) {
      C[i] = (CM3 * (J[i]*J[i]*J[i]) + CM2 * (J[i]*J[i]) + CM1 * J[i] + CM0) /
             (CT3 * (J[i]*J[i]*J[i]) + CT2 * (J[i]*J[i]) + CT1 * J[i] + CT0);
    }

    double sumC = C[0] + C[1] + C[2] + C[3];
    output.force[0] = std::max(((C[1] + C[3])/2 * input.wrench.force.z - (C[1] - C[3])/(2*l_m)          * input.wrench.torque.x - (C[1] + 2*C[2] + C[3])/(2*l_m) * input.wrench.torque.y + input.wrench.torque.z)/sumC, 0.0);
    output.force[1] = std::max(((C[0] + C[2])/2 * input.wrench.force.z - (C[0] + C[2] + 2*C[3])/(2*l_m) * input.wrench.torque.x - (C[0] - C[2])/(2*l_m)          * input.wrench.torque.y - input.wrench.torque.z)/sumC, 0.0);
    output.force[2] = std::max(((C[1] + C[3])/2 * input.wrench.force.z - (C[1] - C[3])/(2*l_m)          * input.wrench.torque.x + (2*C[0] + C[1] + C[3])/(2*l_m) * input.wrench.torque.y + input.wrench.torque.z)/sumC, 0.0);
    output.force[3] = std::max(((C[0] + C[2])/2 * input.wrench.force.z + (C[0] + 2*C[1] + C[2])/(2*l_m) * input.wrench.torque.x - (C[0] - C[2])/(2*l_m)          * input.wrench.torque.y - input.wrench.torque.z)/sumC, 0.0);
    output.torque[0] = C[0] * output.force[0];
    output.torque[1] = C[1] * output.force[1];
    output.torque[2] = C[2] * output.force[2];
    output.torque[3] = C[3] * output.force[3];

  //  output.voltage[0] = R_A / Psi * output.torque[0] + Psi * controller->motor_status.frequency[0];
  //  output.voltage[1] = R_A / Psi * output.torque[1] + Psi * controller->motor_status.frequency[1];
  //  output.voltage[2] = R_A / Psi * output.torque[2] + Psi * controller->motor_status.frequency[2];
  //  output.voltage[3] = R_A / Psi * output.torque[3] + Psi * controller->motor_status.frequency[3];

    static const double c0 = 0.0137, c1 = -0.0117, c2 = 4.0 * c0 / rho / M_PI / (R_P*R_P);
    output.frequency[0] = c1 * M_PI * v1[0];
    output.frequency[0] = (-output.frequency[0] + sqrt(output.frequency[0]*output.frequency[0] + output.force[0] * c2)) / 2.0 / c0 / R_P;
    output.frequency[1] = c1 * M_PI * v1[1];
    output.frequency[1] = (-output.frequency[1] + sqrt(output.frequency[1]*output.frequency[1] + output.force[1] * c2)) / 2.0 / c0 / R_P;
    output.frequency[2] = c1 * M_PI * v1[2];
    output.frequency[2] = (-output.frequency[2] + sqrt(output.frequency[2]*output.frequency[2] + output.force[2] * c2)) / 2.0 / c0 / R_P;
    output.frequency[3] = c1 * M_PI * v1[3];
    output.frequency[3] = (-output.frequency[3] + sqrt(output.frequency[3]*output.frequency[3] + output.force[3] * c2)) / 2.0 / c0 / R_P;

    output.voltage[0] = R_A / Psi * output.torque[0] + Psi * output.frequency[0];
    output.voltage[1] = R_A / Psi * output.torque[1] + Psi * output.frequency[1];
    output.voltage[2] = R_A / Psi * output.torque[2] + Psi * output.frequency[2];
    output.voltage[3] = R_A / Psi * output.torque[3] + Psi * output.frequency[3];

    return true;
  }

} // namespace Controller
} // namespace uxvcos
