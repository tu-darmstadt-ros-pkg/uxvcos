#ifndef TESTBED_H
#define TESTBED_H

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Operation.hpp>
#include <sensor_msgs/typekit/Imu.h>
#include <sensors/Sensor.h>

#include <base/Transformation.h>
#include <filter/PT1.h>

#include <hudaqlib.h>

namespace uxvcos {

class Testbed : public RTT::TaskContext {
public:
  Testbed(const std::string& name = "Testbed");
  virtual ~Testbed();

  virtual bool setZeroCommandExecute(double seconds);

protected:
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();

protected:
  virtual void setZeroCommandUpdate(double values[]);
  virtual void setZeroCommandFinish();

  Sensors::Sensor<sensor_msgs::Imu> reference;

  LinearTransformation<double,double> Encoder0, Encoder1, Encoder2;
  Filter::PT1 Filter0, Filter1, Filter2;

  bool setZeroInProgress;
  RTT::os::TimeService::ticks setZeroTimeout;
  unsigned int setZeroCount;
  double setZeroBuffer[3];

private:
  HUDAQHANDLE h;
};

} // namespace uxvcos

#endif // TESTBED_H

