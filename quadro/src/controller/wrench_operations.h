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

#ifndef QUADRO_CONTROLLER_WRENCH_OPERATIONS_H
#define QUADRO_CONTROLLER_WRENCH_OPERATIONS_H

#include <geometry_msgs/WrenchStamped.h>

static inline geometry_msgs::Wrench operator+(const geometry_msgs::Wrench& op1, const geometry_msgs::Wrench& op2) {
  geometry_msgs::Wrench result;
  result.force.x  = op1.force.x  + op2.force.x;
  result.force.y  = op1.force.y  + op2.force.y;
  result.force.z  = op1.force.z  + op2.force.z;
  result.torque.x = op1.torque.x + op2.torque.x;
  result.torque.y = op1.torque.y + op2.torque.y;
  result.torque.z = op1.torque.z + op2.torque.z;
  return result;
}

static inline geometry_msgs::Wrench operator-(const geometry_msgs::Wrench& op1, const geometry_msgs::Wrench& op2) {
  geometry_msgs::Wrench result;
  result.force.x  = op1.force.x  - op2.force.x;
  result.force.y  = op1.force.y  - op2.force.y;
  result.force.z  = op1.force.z  - op2.force.z;
  result.torque.x = op1.torque.x - op2.torque.x;
  result.torque.y = op1.torque.y - op2.torque.y;
  result.torque.z = op1.torque.z - op2.torque.z;
  return result;
}

static inline geometry_msgs::WrenchStamped operator+(const geometry_msgs::WrenchStamped& op1, const geometry_msgs::WrenchStamped& op2) {
  geometry_msgs::WrenchStamped result;
  result.wrench = op1.wrench + op2.wrench;
  return result;
}

static inline geometry_msgs::WrenchStamped operator-(const geometry_msgs::WrenchStamped& op1, const geometry_msgs::WrenchStamped& op2) {
  geometry_msgs::WrenchStamped result;
  result.wrench = op1.wrench - op2.wrench;
  return result;
}

#endif // QUADRO_CONTROLLER_WRENCH_OPERATIONS_H
