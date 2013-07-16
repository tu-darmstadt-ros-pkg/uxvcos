#ifndef UXVCOS_FILTER_BUTTERWORTH_H
#define UXVCOS_FILTER_BUTTERWORTH_H

#include "BaseFilter.h"
#include <rtt/Property.hpp>

#include <vector>
#include <stdexcept>

namespace uxvcos {
namespace Filter {

template <typename T = double>
class Butterworth : public BaseFilter<T>
{
protected:
    std::vector<T> _x;
    std::vector<T> _a;
    std::vector<int> _b;
    std::vector<T> _c;
    T _dy;
    double _T1;
    double _T1_property;

    using BaseFilter<T>::_dt;

public:
    Butterworth(const std::string& name, double T1, unsigned int order = 2, T initial_value = nan<T>())
      : BaseFilter<T>(name, "Butterworth filter of n-th order")
      , _x(order), _a(order+1), _b(order+1), _c(order+1), _T1(T1), _T1_property(T1)
    {
      if (!computeCoefficients(order)) throw std::invalid_argument("not implemented");
      reset(initial_value);

      this->addProperty("T", "Time constant", _T1_property);
    }
    virtual ~Butterworth() {}

    virtual void reset(T initial_value = nan<T>()) {
        BaseFilter<T>::reset();
        _dy = 0;
        for(unsigned int i = 0; i < _x.size(); ++i) _x[i]  = initial_value;
    }

    virtual T y() const        { return _x[0]; }
    virtual T dy() const       { return _dy; }

    virtual void dt(double dt) {
      _dt = dt;
      computeCoefficients();
    }

    virtual double T1() const   { return _T1; }
    virtual bool setT1(double T1) {
      if (T1 < 0.0) return false;
      _T1 = T1;
      _T1_property = T1;
      return computeCoefficients();
    }

    virtual unsigned int order() const { return _x.size(); }
    virtual bool setOrder(unsigned int order) {
      if (!computeCoefficients(order)) return false;
      reset(y());
      return true;
    }

    using BaseFilter<T>::update;
    bool update(T u) {
      if (isnan(_x[0])) reset(u);
      if (_T1 != _T1_property) { _T1 = _T1_property; computeCoefficients(); }
      if (isnan(u)) return false;

      unsigned int i;
      T result = u;
      for(i = 0; i < _x.size(); ++i) result -= _c[i + 1] * _x[i];
      result = result / _c[0];

      _dy = (result - _x[0]) / _dt;

      // Unit step
      for(i = 1; i < _x.size(); ++i) _x[i] = _x[i-1];

      _x[0] = result;

      return true;
    }

private:
    bool computeCoefficients() {
      return computeCoefficients(_x.size());
    }

    bool computeCoefficients(unsigned int order) {
      unsigned int i,j;
      double dti = 1.0;
      double T1i = 1.0;

      if (order < 1 || order > 4) return false;

      _x.resize(order);
      _a.assign(order+1,0);
      _b.assign(order+1,0);
      _c.assign(order+1,0);

      _a[0] = 1.0;
      switch(_x.size()) {
      case 1:
          _a[1] = 1.0;
          break;
      case 2:
          _a[2] = 1.0;
          _a[1] = 1.4142;
          break;
      case 3:
          _a[3] = 1.0;
          _a[2] = 2.0;
          _a[1] = 2.0;
          break;
      case 4:
          _a[4] = 1.0;
          _a[3] = 2.6132;
          _a[2] = 1.4142;
          _a[1] = 2.6132;
          break;
      };

      _b[0] = 1;
      for(i = 0; i <= _x.size(); ++i) {
        if (i > 0) for(j = i; j > 0; --j)  _b[j] = _b[j] - _b[j-1];
        for(j = 0; j <= i; ++j) _c[j] += _a[i] * static_cast<T>(_b[j]) * T1i / dti;
        dti = dti * _dt;
        T1i = T1i * _T1;
      }

      if (_dt == 0.0) {
        for(j = 0; j <= _x.size(); ++j) _c[j] = 0;
        _c[0] = 1.0;
      }

      return true;
    }
};

} // namespace Filter
} // namespace uxvcos

#endif // UXVCOS_FILTER_BUTTERWORTH_H
