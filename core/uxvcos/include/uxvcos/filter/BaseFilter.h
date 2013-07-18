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

#ifndef UXVCOS_FILTER_BASEFILTER_H
#define UXVCOS_FILTER_BASEFILTER_H

#include <uxvcos/Child.h>
#include <limits>
#include <cmath>

namespace uxvcos {
namespace Filter {

template <typename T> static inline T nan() { return std::numeric_limits<T>::quiet_NaN(); }
template <typename T> static inline bool isnan(const T& value) { return !(value == value); }

template <typename OutputType = double, typename InputType = OutputType>
class BaseFilter : public Child {
public:
  typedef OutputType Output;
  typedef InputType Input;

  BaseFilter(const std::string& name, const std::string& description = "")
    : Child(name, description), _dt(0), _t1(0) {}
  virtual ~BaseFilter() {}

  virtual void reset(Output initial_value = nan<Output>()) {
    _t1 = 0;
  }

  virtual Output y() const = 0;
  virtual Output dy() const = 0;
  virtual double dt() const { return _dt; }
  virtual void dt(double dt) { _dt = dt; }
  virtual double t() const { return _t1; }

  Output operator()(Input u, double dt) {
    update(u, dt);
    return y();
  }

  Output operator()(Input u) {
    update(u, dt());
    return y();
  }

  virtual double timeSinceLastUpdate(double t) const {
    if (t == 0.0 || _t1 == 0.0) return 0.0;
    return t - _t1;
  }

  virtual Output set(Input u, double t) {
    if (update(u, timeSinceLastUpdate(t))) {
      _t1 = t;
    }
    return y();
  }

  virtual bool update(Input u, double dt) {
    this->dt(dt);
    return update(u);
  }

  virtual bool update(Input u) {
    return update(u, dt());
  }

  virtual bool isValid() {
    return !isnan(y());
  }

protected:
  double _dt;
  double _t1;
};

} // namespace Filter
} // namespace uxvcos

#endif // UXVCOS_FILTER_BASEFILTER_H
