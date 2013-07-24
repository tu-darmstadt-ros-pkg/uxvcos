#include "EthernetInterface.h"
#include "EthernetIMU.h"
#include "DirectFeedback.h"

namespace uxvcos {
namespace Interface {

DirectFeedback::DirectFeedback(EthernetInterface *interface, const std::string &name)
  : Module(interface, name, "Direct feedback of IMU measurements to the motors")
{
  this->addProperty("enabled", enabled);
  this->addProperty("gain", gain);
  this->addProperty("bias", bias);

  this->addOperation("enable", &DirectFeedback::enable, this, RTT::OwnThread);
  this->addOperation("disable", &DirectFeedback::disable, this, RTT::OwnThread);
  this->addOperation("isEnabled", &DirectFeedback::isEnabled, this, RTT::ClientThread);
}

DirectFeedback::~DirectFeedback()
{
}

bool DirectFeedback::initialize()
{
  if (!enabled) return true;
  return configure(true);
}

void DirectFeedback::execute()
{
  if (enabled && !isEnabled())
    configure(true);
}

void DirectFeedback::cleanup()
{
  disable();
}

bool DirectFeedback::configure(bool enabled)
{
  feedback.enabled = enabled;

  feedback.bias.accelX = bias[0];
  feedback.bias.accelY = bias[1];
  feedback.bias.accelZ = bias[2];
  feedback.bias.omegaX = bias[3];
  feedback.bias.omegaY = bias[4];
  feedback.bias.omegaZ = bias[5];

  for(size_t i = 0; i < ARMMOTOR_COUNT; i++) {
    feedback.gain[i].accelX = gain[i*6 + 0];
    feedback.gain[i].accelY = gain[i*6 + 1];
    feedback.gain[i].accelZ = gain[i*6 + 2];
    feedback.gain[i].omegaX = gain[i*6 + 3];
    feedback.gain[i].omegaY = gain[i*6 + 4];
    feedback.gain[i].omegaZ = gain[i*6 + 5];
  }

  bool result = getInterface()->send(&feedback, sizeof(feedback), ARM_INTERFACE_CLASS, ARM_CONFIG_DIRECT_FEEDBACK_ID);
  if (!result) {
    feedback.enabled = false;
    RTT::log(RTT::Error) << "Could not configure direct feedback mode." << RTT::endlog();
    return false;
  }

  return getInterface()->receiveAnswer() && (isEnabled() == enabled);
}

void DirectFeedback::receivedResponse(QuadroDirectFeedback_t *response)
{
  RTT::log(RTT::Info) << "Received direct feedback response: " << (response->enabled ? "enabled" : "disabled") << RTT::endlog();
  feedback = *response;
}

bool DirectFeedback::enable()
{
  return configure(true);
}

bool DirectFeedback::disable()
{
  return configure(false);
}

bool DirectFeedback::isEnabled()
{
  return feedback.enabled;
}

//bool DirectFeedback::newMotorCommand(const hector_uav_msgs::MotorPWM &motorCommand)
//{
//  if (!isEnabled()) {
//    return getInterface()->newMotorCommand(motorCommand);
//  }

//  boost::shared_ptr<EthernetIMU> imu = getInterface()->getModule<EthernetIMU>("IMU");
//  if (!imu) {
//    RTT::log(RTT::Error) << "Disabling direct feedback as no IMU is available in this context." << RTT::endlog();
//    disable();
//    return getInterface()->newMotorCommand(motorCommand);
//  }

//  hector_uav_msgs::MotorPWM motorDirectFeedback = motorCommand;
//  for(size_t i = 0; i < motorDirectFeedback.pwm.size() && i < ARMMOTOR_COUNT; ++i) {
//    if (motorDirectFeedback.pwm[i] == 0) continue;

//    int imuGain;
//    imuGain = static_cast<int>(feedback.gain[i].accelX * imu->get().linear_acceleration.x
//                + feedback.gain[i].accelY * imu->get().linear_acceleration.y
//                + feedback.gain[i].accelZ * imu->get().linear_acceleration.z
//                + feedback.gain[i].omegaX * imu->get().angular_velocity.x
//                + feedback.gain[i].omegaY * imu->get().angular_velocity.y
//                + feedback.gain[i].omegaZ * imu->get().angular_velocity.z
//                + 0.5);

//    if (motorDirectFeedback.pwm[i] - imuGain > 255)
//      motorDirectFeedback.pwm[i] = 255;
//    else if (motorDirectFeedback.pwm[i] - imuGain < 20)
//      motorDirectFeedback.pwm[i] = 20;
//    else
//      motorDirectFeedback.pwm[i] -= imuGain;
//  }

//  return getInterface()->newMotorCommand(motorDirectFeedback);
//}

} // namespace Interface
} // namespace uxvcos
