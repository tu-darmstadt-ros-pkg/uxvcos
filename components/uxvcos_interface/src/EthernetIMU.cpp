#include "EthernetIMU.h"
#include "EthernetInterface.h"

#include <limits>
#include "ublox.h"

namespace uxvcos {
namespace Interface {

EthernetIMU::EthernetIMU(EthernetInterface *interface, const std::string& name, const std::string& port_name, const std::string& description)
  : EthernetSensor<Sensors::IMU>(interface, name, port_name, description)
{
  this->addProperty("RawMode", modeRaw).doc("If true, raw values are requested from the ARM Interface");
  this->addOperation("sendConfiguration", &EthernetIMU::sendConfiguration, this).doc("Sends the current configuration to the interface board");
}

EthernetIMU::~EthernetIMU()
{}

bool EthernetIMU::initialize() {
  sendConfiguration();
  return EthernetSensor<Sensors::IMU>::initialize();
}

bool EthernetIMU::sendConfiguration() {
  struct ArmIMUConfig_t imuConfig;
  static const float NaN = std::numeric_limits<float>::quiet_NaN();

  imuConfig.mappingAccel[0]  = static_cast<unsigned char>(mapAccel[0]);
  imuConfig.mappingAccel[1]  = static_cast<unsigned char>(mapAccel[1]);
  imuConfig.mappingAccel[2]  = static_cast<unsigned char>(mapAccel[2]);
  imuConfig.accel[0].offsetA = static_cast<int>(Accel0.offsetA);
  imuConfig.accel[0].factorB = static_cast<float>(Accel0.factorB);
  imuConfig.accel[0].offsetC = static_cast<float>(Accel0.offsetC);
  imuConfig.accel[0].min     = Accel0.limit ? static_cast<float>(Accel0.minimum) : NaN;
  imuConfig.accel[0].max     = Accel0.limit ? static_cast<float>(Accel0.maximum) : NaN;
  imuConfig.accel[1].offsetA = static_cast<int>(Accel1.offsetA);
  imuConfig.accel[1].factorB = static_cast<float>(Accel1.factorB);
  imuConfig.accel[1].offsetC = static_cast<float>(Accel1.offsetC);
  imuConfig.accel[1].min     = Accel1.limit ? static_cast<float>(Accel1.minimum) : NaN;
  imuConfig.accel[1].max     = Accel1.limit ? static_cast<float>(Accel1.maximum) : NaN;
  imuConfig.accel[2].offsetA = static_cast<int>(Accel2.offsetA);
  imuConfig.accel[2].factorB = static_cast<float>(Accel2.factorB);
  imuConfig.accel[2].offsetC = static_cast<float>(Accel2.offsetC);
  imuConfig.accel[2].min     = Accel2.limit ? static_cast<float>(Accel2.minimum) : NaN;
  imuConfig.accel[2].max     = Accel2.limit ? static_cast<float>(Accel2.maximum) : NaN;

  imuConfig.mappingGyro[0]  = static_cast<unsigned char>(mapGyro[0]);
  imuConfig.mappingGyro[1]  = static_cast<unsigned char>(mapGyro[1]);
  imuConfig.mappingGyro[2]  = static_cast<unsigned char>(mapGyro[2]);
  imuConfig.gyro[0].offsetA = static_cast<int>(Gyro0.offsetA);
  imuConfig.gyro[0].factorB = static_cast<float>(Gyro0.factorB);
  imuConfig.gyro[0].offsetC = static_cast<float>(Gyro0.offsetC);
  imuConfig.gyro[0].min     = Gyro0.limit ? static_cast<float>(Gyro0.minimum) : NaN;
  imuConfig.gyro[0].max     = Gyro0.limit ? static_cast<float>(Gyro0.maximum) : NaN;
  imuConfig.gyro[1].offsetA = static_cast<int>(Gyro1.offsetA);
  imuConfig.gyro[1].factorB = static_cast<float>(Gyro1.factorB);
  imuConfig.gyro[1].offsetC = static_cast<float>(Gyro1.offsetC);
  imuConfig.gyro[1].min     = Gyro1.limit ? static_cast<float>(Gyro1.minimum) : NaN;
  imuConfig.gyro[1].max     = Gyro1.limit ? static_cast<float>(Gyro1.maximum) : NaN;
  imuConfig.gyro[2].offsetA = static_cast<int>(Gyro2.offsetA);
  imuConfig.gyro[2].factorB = static_cast<float>(Gyro2.factorB);
  imuConfig.gyro[2].offsetC = static_cast<float>(Gyro2.offsetC);
  imuConfig.gyro[2].min     = Gyro2.limit ? static_cast<float>(Gyro2.minimum) : NaN;
  imuConfig.gyro[2].max     = Gyro2.limit ? static_cast<float>(Gyro2.maximum) : NaN;

  return interface->send(&imuConfig, sizeof(imuConfig), ARM_INTERFACE_CLASS, ARM_CONFIG_IMU_ID);
}

bool EthernetIMU::setZeroCommandExecute(double seconds) {
  if (!modeRaw) {
    RTT::log(RTT::Error) << "setZero is only available in RawMode!" << RTT::endlog();
    return false;
  }
  return Sensors::IMU::setZeroCommandExecute(seconds);
}

void EthernetIMU::addTo(EthernetSensorContainer &sensors) {
  sensors.insert(ARM_RAW_IMU_ID, sensor());
  sensors.insert(ARM_IMU_ID, sensor());
}

void EthernetIMU::request(Request &request) const {
  if (modeRaw)
    SET_REQUEST(ARM_RAW_IMU_ID, request);
  else
    SET_REQUEST(ARM_IMU_ID, request);
}

// this method will be overridden by the following one (this is an ugly workaround)
template<> bool EthernetSensor<Sensors::IMU>::decode(void *, size_t, MessageId) {
  return false;
}

bool EthernetIMU::decode(void *payload, size_t length, MessageId id) {
  if (modeRaw && id == ARM_RAW_IMU_ID && length >= sizeof(ArmRawIMU_t))
  {
    struct ArmRawIMU_t *ArmImu = reinterpret_cast<struct ArmRawIMU_t *>(payload);

    raw.linear_acceleration[0] = ArmImu->accel[0];
    raw.linear_acceleration[1] = ArmImu->accel[1];
    raw.linear_acceleration[2] = ArmImu->accel[2];
    raw.angular_velocity[0]    = ArmImu->omega[0];
    raw.angular_velocity[1]    = ArmImu->omega[1];
    raw.angular_velocity[2]    = ArmImu->omega[2];

    raw.header.stamp = interface->getTimestamp();

    this->updatedRaw();
    return true;
  }

  if (!modeRaw && id == ARM_IMU_ID && length >= sizeof(ArmIMU_t))
  {
    const struct ArmIMU_t *ArmImu = reinterpret_cast<const struct ArmIMU_t *>(payload);

    data.linear_acceleration.x = ArmImu->accelX;
    data.linear_acceleration.y = ArmImu->accelY;
    data.linear_acceleration.z = ArmImu->accelZ;
    data.angular_velocity.x    = ArmImu->gyroX;
    data.angular_velocity.y    = ArmImu->gyroY;
    data.angular_velocity.z    = ArmImu->gyroZ;

    data.header.stamp = interface->getTimestamp();

    this->updated();
    return true;
  }

  return false;
}

std::ostream& EthernetIMU::operator >>(std::ostream& os) {
  char debugBuffer[256];
  snprintf(debugBuffer, sizeof(debugBuffer) - 1, "AccelX:%.3f AccelY:%.3f AccelZ:%.3f GyroX:%.3f GyroY:%.3f GyroZ:%.3f",
           data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z, data.angular_velocity.x*180.0/M_PI, data.angular_velocity.y*180.0/M_PI, data.angular_velocity.z*180.0/M_PI);
  return os << std::string(debugBuffer);
}

} // namespace Interfacee
} // namespace uxvcos
