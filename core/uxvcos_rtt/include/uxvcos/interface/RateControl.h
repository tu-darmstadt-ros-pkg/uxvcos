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

#ifndef INTERFACE_RATECONTROL_H
#define INTERFACE_RATECONTROL_H

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>

namespace uxvcos {

class RateControl
{
public:
  typedef unsigned long long counter_t;

  RateControl(RTT::PropertyBag *bag, RateControl* reference = 0)
    : reference(reference), my_counter(0), my_period(0.0)
    , rate("rate",     "Measurement rate in milliseconds", 0)
    , offset("offset", "Measurement offset in milliseconds", 0)
  {
    bag->addProperty(rate);
    bag->addProperty(offset);
  }

  RateControl(RateControl* reference = 0)
    : reference(reference), my_counter(0), my_period(0.0)
    , rate("rate",     "Measurement rate in milliseconds", 0)
    , offset("offset", "Measurement offset in milliseconds", 0)
  {
  }

  virtual ~RateControl() {}

  counter_t &counter() {
    if (reference) return reference->counter();
    return my_counter;
  }

  const counter_t &counter() const {
    if (reference) return reference->counter();
    return my_counter;
  }

  double &period() {
    if (reference) return reference->period();
    return my_period;
  }

  const double &period() const {
    if (reference) return reference->period();
    return my_period;
  }

  counter_t &step() {
    return ++(counter());
  }

  operator bool() const {
    return isTime();
  }

  bool isTime() const
  {
    return isTime(counter());
  }

  bool isTime(counter_t counter) const
  {
    if (rate == 0) return false;
    if (rate == 1) return true;
    if (period() == 0.0) return false;

    const double frequency = 0.001 / period(); // in kHz
    unsigned int divisor   = static_cast<unsigned int>(ceil(static_cast<double>(rate.get()) * frequency)); // aufrunden
    unsigned int remainder = static_cast<unsigned int>(floor(static_cast<double>(offset.get()) * frequency)); // abrunden

//     RTT::log( RTT::Debug)
//       << "frequency    = " << frequency << " kHz" << RTT::nlog()
//       << "rate         = " << rate.get() << " ms" << RTT::nlog()
//       << "=> divisor   = " << divisor << RTT::nlog()
//       << "offset       = " << offset.get() << " ms" << RTT::nlog()
//       << "=> remainder = " << remainder << RTT::endlog();

    return (counter % divisor == remainder);
  }

private:
  RateControl* reference;

protected:
  counter_t my_counter;
  double my_period;

public:
  RTT::Property<unsigned int> rate;
  RTT::Property<unsigned int> offset;
};

} // namespace uxvcos

#endif // INTERFACE_RATECONTROL_H
