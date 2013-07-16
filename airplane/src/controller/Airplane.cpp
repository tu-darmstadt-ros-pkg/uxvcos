#include "Airplane.h"

#include <rtt/Logger.hpp>
#include <controller/Limit.h>

namespace uxvcos {
namespace Controller {

Airplane::Airplane(const std::string &name)
  : ControllerTask(name)
  , rc(this)
  , throttle(this)
  , rudders(this)
{
  this->addModule(&throttle);
  this->addModule(&rudders);
}

Airplane::~Airplane() {
  stop();
  cleanup();
}

void Airplane::reset() {
  throttle.reset();
  rudders.reset();
}

Airplane::Throttle::Throttle(Airplane* controller, const std::string& name, const std::string& description)
  : Controller<Airplane, hector_uav_msgs::MotorPWM, hector_std_msgs::Float32>(controller, name, description)
  , on("on")
{
  controller->provides()->addAttribute(on);
}

bool Airplane::Throttle::update(double dt) {
  const Limit<double> limit(10,120);

  reset();
  if (on.get()) {
    output.pwm[0] = static_cast<unsigned char>(limit(120 * input.data));
  }

  return true;
}

void Airplane::Throttle::reset() {
  output.pwm[0] = 0;
  output.pwm[1] = 0;
  output.pwm[2] = 0;
  output.pwm[3] = 0;
}

bool Airplane::Rudders::update(double dt) {
  for(std::size_t i = 0; i < output.value.size(); i++) output.value[i] = 0;
  
  if (leftaileron.channel > 0 && leftaileron.channel <= output.value.size())
    output.value[leftaileron.channel-1]  = static_cast<uint16_t>(1500.0 + 400.0 * leftaileron.transformation(input.aileron));

  if (rightaileron.channel > 0 && rightaileron.channel <= output.value.size())
    output.value[rightaileron.channel-1] = static_cast<uint16_t>(1500.0 + 400.0 * rightaileron.transformation(input.aileron));

  if (elevator.channel > 0 && elevator.channel <= output.value.size())
    output.value[elevator.channel-1]     = static_cast<uint16_t>(1500.0 + 400.0 * elevator.transformation(input.elevator));

  if (rudder.channel > 0 && rudder.channel <= output.value.size())
    output.value[rudder.channel-1]       = static_cast<uint16_t>(1500.0 + 400.0 * rudder.transformation(input.rudder));

  if (throttle.channel > 0 && throttle.channel <= output.value.size())
    output.value[throttle.channel-1]     = static_cast<uint16_t>(1500.0 + 400.0 * (2 * throttle.transformation(getOwner()->throttle.getOutput().pwm[0]) - 1));

  return true;
}

void Airplane::Rudders::reset() {
}

} // namespace Controller
} // namespace uxvcos
