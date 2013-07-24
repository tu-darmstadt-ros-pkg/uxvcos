#ifndef UXVCOS_INTERFACE_DIRECTFEEDBACK_H
#define UXVCOS_INTERFACE_DIRECTFEEDBACK_H

#include <uxvcos/Module.h>
#include "Motor.h"

#include "arm-interface.h"
#include <sensor_msgs/Imu.h>

namespace uxvcos {
namespace Interface {

class EthernetInterface;

class DirectFeedback : public Module /* , public Motor::Handler */
{
public:
  DirectFeedback(EthernetInterface* parent, const std::string& name = "Direct Feedback");
  virtual ~DirectFeedback();

  EthernetInterface *getInterface() { return dynamic_cast<EthernetInterface *>(this->getOwner()); }

  bool initialize();
  void execute();
  void cleanup();

  bool configure(bool enabled);
  bool enable();
  bool disable();
  bool isEnabled();

//  bool newMotorCommand(const hector_uav_msgs::MotorPWM &motorCommand);
  void receivedResponse(QuadroDirectFeedback_t *response);

private:
  QuadroDirectFeedback_t feedback;

  bool enabled;
  boost::array<double,6 * ARMMOTOR_COUNT> gain;
  boost::array<double,6> bias;
};

} // namespace Interface
} // namespace uxvcos

#endif // UXVCOS_INTERFACE_DIRECTFEEDBACK_H
