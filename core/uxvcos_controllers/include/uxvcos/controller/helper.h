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
