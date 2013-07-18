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

#ifndef UXVCOS_CONTROLLER_HELPER_H
#define UXVCOS_CONTROLLER_HELPER_H

#include <math.h>
#include <stdlib.h>
#include <limits>

#ifndef __GNUC__
	#define isnan _isnan
#endif

namespace uxvcos {
namespace Controller {
namespace helper {

static inline double NaN() {
  return std::numeric_limits<double>::quiet_NaN();
}

static inline double diff(const double x, double& x_1, const double dt) {
  double d = 0.0;
  if (dt > 0.0 && !isnan(x_1)) {
    d = (x - x_1) / dt;
  }
  x_1 = x;
  return d;
}

static inline double modulo(const double x, const double min, const double max) {
  const double d = max - min;
  return x - floor((x - min)/d) * d;
}

//double PT1(const double u, const double y_1, const double T, double *diff);
static inline double PT1(const double u, double y_1, const double T, const double dt, double * const pdy) {
  if (isnan(y_1)) y_1 = u;
  if (dt <= 0.0) return y_1;
  if (pdy != NULL) *pdy = (u - y_1) / (dt + T);
  return (dt * u + T * y_1) / (dt + T);
}

static inline double PT1_modulo(const double u, double y_1, const double T, const double dt, double * const pdy, const double min, const double max) {
  if (isnan(y_1)) y_1 = u;
  if (dt <= 0.0) return y_1;
  double dy = modulo(u - y_1, min, max) / (dt + T);
  if (pdy != NULL) *pdy = dy;
  return modulo(y_1 + dt * dy, min, max);
}

static inline double limit2(const double x, const double min, const double max) {
  if (isnan(x)) return 0.0;
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

static inline double limit1(const double x, const double limit) {
  return limit2(x, -limit, limit);
}

} // namespace helper
} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_HELPER_H
