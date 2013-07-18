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
