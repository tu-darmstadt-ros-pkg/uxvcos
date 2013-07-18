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
