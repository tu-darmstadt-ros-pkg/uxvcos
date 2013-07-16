#ifndef UXVCOS_TRANSFORMATION_H
#define UXVCOS_TRANSFORMATION_H

#include <base/Child.h>

#include <string>
#include <cfloat>
#include <limits>

template <typename T>
static T add(T a, T b) {
  static T min = std::numeric_limits<T>::min();
  static T max = std::numeric_limits<T>::max();

  if (!std::numeric_limits<T>::is_modulo) return a + b;
  if (b > 0 && a > max - b) return max;
  if (b < 0 && a < min - b) return min;
  return a + b;
}

template <typename T>
static T sub(T a, T b) {
  static T min = std::numeric_limits<T>::min();
  static T max = std::numeric_limits<T>::max();

  if (!std::numeric_limits<T>::is_modulo) return a - b;
  if (b < 0 && a > max + b) return max;
  if (b > 0 && a < min + b) return min;
  return a - b;
}

namespace uxvcos {

template <typename InputType, typename OutputType>
    class Transformation : public Child {
public:
  Transformation(const std::string& name = "Transformation", const std::string& description = "")
    : Child(name, description)
    , limit("limit", "If true, values are limited between limitMin and limitMax", false)
    , minimum("min", "Lower limit for the output value")
    , maximum("max", "Upper limit for the output value")
  {
    this->declareProperty(limit);
    this->declareProperty(minimum);
    this->declareProperty(maximum);
  }

  Transformation(RTT::PropertyBag* bag, const std::string& name = "Transformation", const std::string& description = "")
    : Child(bag, name, description)
    , limit("limit", "If true, values are limited between limitMin and limitMax", false)
    , minimum("min", "Lower limit for the output value")
    , maximum("max", "Upper limit for the output value")
  {
    this->declareProperty(limit);
    this->declareProperty(minimum);
    this->declareProperty(maximum);
  }

  virtual ~Transformation() {
  }

  virtual OutputType convert(const InputType& raw) const = 0;
  virtual InputType invert(const OutputType& out) const = 0;
  OutputType operator()(const InputType& raw) const { return convert(raw); }

public:
  RTT::Property<bool> limit;
  RTT::Property<OutputType> minimum;
  RTT::Property<OutputType> maximum;
};

template <typename InputType, typename OutputType>
class LinearTransformation : public Transformation<InputType, OutputType> {
public:
  LinearTransformation(const std::string& name = "LinearTransformation", const std::string& description = "")
    : Transformation<InputType, OutputType>(name, description)
    , offsetA("offsetA", "This offset will be added before the multiplication", 0)
    , factorB("factorB", "The corrected value will be multiplied by this value", 1)
    , offsetC("offsetC", "This offset will be added after the multiplication", 0)
  {
    this->declareProperty(offsetA);
    this->declareProperty(factorB);
    this->declareProperty(offsetC);
  }

  LinearTransformation(RTT::PropertyBag* bag)
    : Transformation<InputType, OutputType>(bag)
    , offsetA("offsetA", "This offset will be added before the multiplication", 0)
    , factorB("factorB", "The corrected value will be multiplied by this value", 1)
    , offsetC("offsetC", "This offset will be added after the multiplication", 0)
  {
    this->declareProperty(offsetA);
    this->declareProperty(factorB);
    this->declareProperty(offsetC);
  }

  virtual OutputType convert(const InputType& raw) const {
    OutputType value = add(static_cast<OutputType>(static_cast<double>(add(raw, offsetA.get())) * factorB.get()), offsetC.get());
    if (this->limit.get() && value < this->minimum.get()) value = this->minimum.get();
    if (this->limit.get() && value > this->maximum.get()) value = this->maximum.get();
    return value;
  }

  virtual InputType invert(const OutputType& out) const {
    return sub(static_cast<InputType>(static_cast<double>(sub(out, offsetC.get())) / factorB.get()), offsetA.get());
  }

public:
  RTT::Property<InputType>  offsetA;
  RTT::Property<double> factorB;
  RTT::Property<OutputType> offsetC;
};

template <typename InputType, typename OutputType>
class QuadraticTransformation : public Transformation<InputType, OutputType> {
public:
  QuadraticTransformation(const std::string& name = "QuadraticTransformation", const std::string& description = "")
    : Transformation<InputType, OutputType>(name, description)
    , offsetRaw("offsetRaw", "This offset will be added before the transformation", 0)
    , a("a", "Constant a in a * x^2 + b * x + c", 0)
    , b("b", "Constant b in a * x^2 + b * x + c", 1)
    , c("c", "Constant c in a * x^2 + b * x + c", 0)
  {
    this->declareProperty(offsetRaw);
    this->declareProperty(a);
    this->declareProperty(b);
    this->declareProperty(c);
  }

  QuadraticTransformation(RTT::PropertyBag* bag)
    : Transformation<InputType, OutputType>(bag)
    , offsetRaw("offsetRaw", "This offset will be added before the transformation", 0)
    , a("a", "Constant a in a * x^2 + b * x + c", 0)
    , b("b", "Constant b in a * x^2 + b * x + c", 1)
    , c("c", "Constant c in a * x^2 + b * x + c", 0)
  {
    this->declareProperty(offsetRaw);
    this->declareProperty(a);
    this->declareProperty(b);
    this->declareProperty(c);
  }

  virtual OutputType convert(const InputType& raw) const {
    double temp = static_cast<double>(add(raw, offsetRaw.get()));
    OutputType value = add(static_cast<OutputType>(temp * temp * a.get() + temp * b.get()), c.get());
    if (this->limit.get() && value < this->minimum.get()) value = this->minimum.get();
    if (this->limit.get() && value > this->maximum.get()) value = this->maximum.get();
    return value;
  }

  virtual InputType invert(const OutputType& out) const {
    double temp = (-b.get() + sqrt(b.get() * b.get() + 4 * a.get() * static_cast<double>(sub(out, c.get())))) / (2 * a.get());
    return sub(static_cast<InputType>(temp), offsetRaw.get());
  }

public:
  RTT::Property<InputType> offsetRaw;
  RTT::Property<double> a;
  RTT::Property<double> b;
  RTT::Property<OutputType> c;
};
} // namespace BASE

#endif // UXVCOS_TRANSFORMATION_H
