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

#ifndef UXVCOS_FILTER_PTN_H
#define UXVCOS_FILTER_PTN_H

#include "BaseFilter.h"
#include <rtt/Property.hpp>

#include <vector>
#include <stdexcept>

namespace uxvcos {
namespace Filter {

template <typename T = double>
class PTn : public BaseFilter<T>
{
protected:
    std::vector<T> _x;
    T _dy;
    double _T1;

    using BaseFilter<T>::_dt;

public:
    PTn(const std::string& name, double T1, unsigned int order = 1, T initial_value = nan<T>())
      : BaseFilter<T>(name, "PTn Low-pass filter of n-th order")
      , _x(order), _T1(T1)
    {
      if (order < 1) throw std::invalid_argument("order must be greater or equal than 1");
      reset(initial_value);

      this->addProperty("T", "Time constant", _T1);
    }
    virtual ~PTn() {}

    virtual void reset(T initial_value = nan<T>()) {
        BaseFilter<T>::reset();
        _dy = 0;
        for(unsigned int i = 0; i < _x.size(); ++i) _x[i]  = initial_value;
    }

    virtual T y() const        { return _x[0]; }
    virtual T dy() const       { return _dy; }

    virtual double T1() const  { return _T1; }
    virtual bool setT1(double T1) {
      if (T1 < 0.0) return false;
      _T1 = T1;
      return true;
    }

    virtual unsigned int order() const { return _x.size(); }
    virtual bool setOrder(unsigned int order) {
      if (order > 0 && order <= 1) {
        _x.resize(order);
        reset(y());
        return true;
      }
      return false;
    }

    using BaseFilter<T>::update;
    virtual bool update(T u) {
      if (isnan(_x[0])) reset(u);
      if (_dt == 0.0) return true;
      if (isnan(u)) return false;

      switch(_x.size()) {
          case 1:
              _dy = (u - _x[0]) / (_dt + _T1);
              _x[0]  = (_dt * u + _T1 * _x[0]) / (_dt + _T1);
              break;
          default:
              // throw std::invalid_argument("not implemented");
              break;
      }

      return true;
    }
};

typedef PTn<double> PT1;

} // namespace Filter
} // namespace uxvcos

#endif // UXVCOS_FILTER_PTN_H
