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

#ifndef UXVCOS_FILTER_OUTLIERELIMINATION_H
#define UXVCOS_FILTER_OUTLIERELIMINATION_H

#include "BaseFilter.h"

namespace uxvcos {
namespace Filter {

template <typename T = double>
class OutlierElimination : public BaseFilter<T>
{
protected:
    T _x;
    T _y;
    T _dy;
    T _max_delta;

    T _current_bound;
    T _current_dt;

    using BaseFilter<T>::_dt;

public:
    OutlierElimination(const std::string& name, T max_delta, T initial_value = nan<T>())
      : BaseFilter<T>(name, "Eliminate outliers that exceed max_delta * dt limit")
      , _x(0), _y(0), _dy(0), _max_delta(max_delta)
    {
      reset(initial_value);
      this->addProperty("max_delta", "Upper bound for the Maximum change per seconds", _max_delta);
    }
    virtual ~OutlierElimination() {}

    virtual void reset(T initial_value = nan<T>()) {
        BaseFilter<T>::reset();
        _x  = initial_value;
        _y  = initial_value;
        _dy = 0;
        _current_bound = 0;
        _current_dt = 0;
    }

    virtual T y() const        { return _y; }
    virtual T dy() const       { return _dy; }

    virtual double max_delta() const { return _max_delta; }
    virtual void max_delta(double max_delta) { _max_delta = fabs(max_delta); }

    using BaseFilter<T>::update;
    bool update(T u) {
      if (isnan(_x)) {
        reset(u);
        return y();
      }

      _current_bound += _dt * _max_delta;
      _current_dt += _dt;

      if (!isnan(_x) && (u < _x - _current_bound || u > _x + _current_bound)) {
        _y  = nan<T>();
        _dy = nan<T>();
        return false;
      }

      _dy = (u - _x) / _current_dt;
      _y = _x = u;
      _current_bound = 0;
      _current_dt = 0;
      return true;
    }
};

} // namespace Filter
} // namespace uxvcos

#endif // UXVCOS_FILTER_OUTLIERELIMINATION_H
