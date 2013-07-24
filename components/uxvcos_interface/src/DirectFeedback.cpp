#include "EthernetInterface.h"
#include "EthernetIMU.h"
#include "DirectFeedback.h"

#include <boost/lexical_cast.hpp>

namespace uxvcos {
namespace Interface {

DirectFeedback::DirectFeedback(EthernetInterface *interface, const std::string &name)
  : Module(interface, name, "Direct feedback of IMU measurements to the motors")
{
  this->addProperty("enabled", enabled);

  bias.ownProperty(new RTT::Property<double>("AccelX", "AccelX bias"));
  bias.ownProperty(new RTT::Property<double>("AccelY", "AccelY bias"));
  bias.ownProperty(new RTT::Property<double>("AccelZ", "AccelZ bias"));
  bias.ownProperty(new RTT::Property<double>("GyroX", "GyroX bias"));
  bias.ownProperty(new RTT::Property<double>("GyroY", "GyroY bias"));
  bias.ownProperty(new RTT::Property<double>("GyroZ", "GyroZ bias"));
  this->addProperty("Bias", bias);

  for(size_t i = 0; i < ARMMOTOR_COUNT; ++i) {
    std::string motor = "Motor" + boost::lexical_cast<std::string>(i+1);
    gain[i].ownProperty(new RTT::Property<double>("AccelX", "Gain for " + motor + "/AccelX"));
    gain[i].ownProperty(new RTT::Property<double>("AccelY", "Gain for " + motor + "/AccelY"));
    gain[i].ownProperty(new RTT::Property<double>("AccelZ", "Gain for " + motor + "/AccelZ"));
    gain[i].ownProperty(new RTT::Property<double>("OmegaX", "Gain for " + motor + "/OmegaX"));
    gain[i].ownProperty(new RTT::Property<double>("OmegaY", "Gain for " + motor + "/OmegaY"));
    gain[i].ownProperty(new RTT::Property<double>("OmegaZ", "Gain for " + motor + "/OmegaZ"));
    this->addProperty(motor, gain[i]);
  }

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

  feedback.bias.accelX = bias.getPropertyType<double>("AccelX")->get();
  feedback.bias.accelY = bias.getPropertyType<double>("AccelY")->get();
  feedback.bias.accelZ = bias.getPropertyType<double>("AccelZ")->get();
  feedback.bias.omegaX = bias.getPropertyType<double>("GyroX")->get();
  feedback.bias.omegaY = bias.getPropertyType<double>("GyroY")->get();
  feedback.bias.omegaZ = bias.getPropertyType<double>("GyroZ")->get();

  for(size_t i = 0; i < ARMMOTOR_COUNT; i++) {
    feedback.gain[i].accelX = gain[i].getPropertyType<double>("AccelX")->get();
    feedback.gain[i].accelY = gain[i].getPropertyType<double>("AccelY")->get();
    feedback.gain[i].accelZ = gain[i].getPropertyType<double>("AccelZ")->get();
    feedback.gain[i].omegaX = gain[i].getPropertyType<double>("OmegaX")->get();
    feedback.gain[i].omegaY = gain[i].getPropertyType<double>("OmegaY")->get();
    feedback.gain[i].omegaZ = gain[i].getPropertyType<double>("OmegaZ")->get();
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
