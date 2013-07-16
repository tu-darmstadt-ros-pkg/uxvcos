#ifndef UXVCOS_SENSOR_IMU_H
#define UXVCOS_SENSOR_IMU_H

#include <sensors/Sensor.h>
#include <base/Transformation.h>
#include <base/Mapping.h>
#include <filter/PTn.h>

#include <hector_uav_msgs/typekit/RawImu.h>
#include <sensor_msgs/typekit/Imu.h>

namespace uxvcos {
namespace Sensors {

class IMU : public RawSensor<hector_uav_msgs::RawImu, sensor_msgs::Imu> {
public:
  enum Axis { ACCELX = 1, ACCELY = 2, ACCELZ = 4, GYROX = 8, GYROY = 16, GYROZ = 32 };

  IMU(RTT::TaskContext* parent, const std::string& name = "IMU", const std::string& port_name = "raw_imu", const std::string& description = "Inertial Measurement Unit")
    : RawSensor<hector_uav_msgs::RawImu, sensor_msgs::Imu>(parent, name, port_name, description)
    , Accel0("Accel0")
    , Accel1("Accel1")
    , Accel2("Accel2")
    , Gyro0("Gyro0")
    , Gyro1("Gyro1")
    , Gyro2("Gyro2")
    , mapAccel(3)
    , mapGyro(3)
    , Filter_AccelX("AccelX", 0.0)
    , Filter_AccelY("AccelY", 0.0)
    , Filter_AccelZ("AccelZ", 0.0)
    , Filter_GyroX("GyroX", 0.0)
    , Filter_GyroY("GyroY", 0.0)
    , Filter_GyroZ("GyroZ", 0.0)
    , setZeroInProgress(false)
    , setZeroBuffer(6)
  {
    properties()->addProperty(Accel0);
    properties()->addProperty(Accel1);
    properties()->addProperty(Accel2);
    properties()->addProperty(Gyro0);
    properties()->addProperty(Gyro1);
    properties()->addProperty(Gyro2);

    properties()->addProperty("MappingAccel", mapAccel).doc("Mapping of the acceleration sensors to the x,y,z axes");
    properties()->addProperty("MappingGyro", mapGyro).doc("Mapping of the gyro sensors to the x,y,z axes");

    properties()->addProperty(Filter_AccelX);
    properties()->addProperty(Filter_AccelY);
    properties()->addProperty(Filter_AccelZ);
    properties()->addProperty(Filter_GyroX);
    properties()->addProperty(Filter_GyroY);
    properties()->addProperty(Filter_GyroZ);

    this->addOperation("setZero", &IMU::setZeroCommandExecute, this, RTT::OwnThread).doc("Remember the current IMU measurements as zero").arg("seconds", "Integration time");
    this->addOperation("setFilterOrder", &IMU::setFilterOrder, this).doc("Set order of the low-pass filter").arg("order", "Order [1,...]");
    this->addOperation("setFilterT", &IMU::setFilterT, this).doc("Set time constant of the low-pass filter").arg("T", "T (in seconds)");
  }
  virtual ~IMU() {}

  virtual bool setFilterOrder(unsigned int order) {
    bool result = true;
    result &= Filter_AccelX.setOrder(order);
    result &= Filter_AccelY.setOrder(order);
    result &= Filter_AccelZ.setOrder(order);
    result &= Filter_GyroX.setOrder(order);
    result &= Filter_GyroY.setOrder(order);
    result &= Filter_GyroZ.setOrder(order);
    return result;
  }

  virtual bool setFilterT(double T1) {
    bool result = true;
    result &= Filter_AccelX.setT1(T1);
    result &= Filter_AccelY.setT1(T1);
    result &= Filter_AccelZ.setT1(T1);
    result &= Filter_GyroX.setT1(T1);
    result &= Filter_GyroY.setT1(T1);
    result &= Filter_GyroZ.setT1(T1);
    return result;
  }

  virtual bool convert(const hector_uav_msgs::RawImu& raw);

  virtual bool setZeroCommandExecute(double seconds, unsigned int axes = 0);

protected:
  // virtual bool setZeroCommandCompleted(double seconds);
  virtual void setZeroCommandUpdate(const hector_uav_msgs::RawImu& raw);
  virtual void setZeroCommandFinish();

protected:
  LinearTransformation<int,double> Accel0, Accel1, Accel2, Gyro0, Gyro1, Gyro2;
  Mapping mapAccel, mapGyro;
  Filter::PTn<double> Filter_AccelX, Filter_AccelY, Filter_AccelZ, Filter_GyroX, Filter_GyroY, Filter_GyroZ;

  bool setZeroInProgress;
  unsigned int setZeroAxes;
  RTT::os::TimeService::ticks setZeroTimeout;
  unsigned int setZeroCount;
  std::vector<double> setZeroBuffer;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSOR_IMU_H
