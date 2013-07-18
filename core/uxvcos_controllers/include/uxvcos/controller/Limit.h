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

#ifndef UXVCOS_CONTROLLER_LIMIT_H
#define UXVCOS_CONTROLLER_LIMIT_H

#include <rtt/types/TemplateCompositionFactory.hpp>
#include <limits>

#undef min
#undef max

namespace uxvcos {
namespace Controller {

template<typename T = double>
class Limit
{
private:
  T min;
  T max;
  mutable int result;

public:
  Limit(T min, T max) : min(min), max(max) {}
  Limit(T minmax) : min(-minmax), max(minmax) {}
  Limit() : min(NaN()), max(NaN()) {}

  static Limit construct2(T min, T max) {
    return Limit(min, max);
  }

  static Limit construct1(T minmax) {
    return Limit(minmax);
  }

  T operator()(T value) const {
    result = 0;
    if (hasMin() && value < min) {
      value = min;
      result = -1;
    } else if (hasMax() && value > max) {
      value = max;
      result = 1;
    }
    return value;
  }
  
  int inLimits() const { return result; }

  bool hasMin() const {
    return (min == min);
  }

  bool hasMax() const {
    return (max == max);
  }

  T Min() const {
    return min;
  }

  T Max() const {
    return max;
  }

  Limit& Min(T min) {
    this->min = min;
    return *this;
  }
  
  Limit& Max(T max) {
    this->max = max;
    return *this;
  }

  static const T NaN() {
    return std::numeric_limits<T>::quiet_NaN();
  }

  friend class CompositionFactory;
  class CompositionFactory : public RTT::types::TemplateCompositionFactory<Limit<T> > {
  public:
    virtual bool decomposeTypeImpl(const Limit<T>& in, RTT::PropertyBag& targetbag) const {
      targetbag.setType("Limit");
      if (!(-in.min == in.max)) {
        if (in.hasMin()) targetbag.add( new RTT::Property<T>("min", "Minimum Value (or NaN)", in.min ) );
        if (in.hasMax()) targetbag.add( new RTT::Property<T>("max", "Maximum Value (or NaN)", in.max ) );
      } else {
	targetbag.add( new RTT::Property<T>("abs", "Maximum absolute value", in.max ) );
      }
      return true;
    }

    virtual bool composeTypeImpl(const RTT::PropertyBag& bag, Limit<T>& out) const 
    {
      if ( bag.getType() == std::string("Controller::Limit") || bag.getType() == std::string("Limit") ) // check the type
      {  
        RTT::Property<T>* min = bag.getPropertyType<T>("min");
        RTT::Property<T>* max = bag.getPropertyType<T>("max");
        RTT::Property<T>* minmax = bag.getPropertyType<T>("abs");

	out.min = min ? min->get() : (minmax ? -minmax->get() : NaN());
	out.max = max ? max->get() : (minmax ?  minmax->get() : NaN());
	return true;
      }
      return false; // unknown type !
    }
  };
};

#include <iostream>

template<typename T>
static inline std::ostream& operator<<(std::ostream& os, const Limit<T>& limit) {
  return os << limit.Min() << "/" << limit.Max();
}

template<typename T>
static inline std::istream& operator>>(std::istream& is, Limit<T>& limit) {
  T min, max;
  char c;

  is >> min;
  limit.Min(min);
  if (is >> c && c == '/' && is >> max) {
    limit.Max(max);
  } else {
    limit.Max(min);
  }
  return is;
}

} // namespace Controller
} // namespace uxvcos

#endif // UXVCOS_CONTROLLER_LIMIT_H
