#ifndef UXVCOS_SENSORS_FMP_H
#define UXVCOS_SENSORS_FMP_H

#include <fmp_msgs/typekit/Types.h>
#include <sensors/Sensor.h>
#include <base/Transformation.h>

#include <limits>

namespace uxvcos {
namespace Sensors {

class FMP : public RawSensor<fmp_msgs::RawData, fmp_msgs::SensorData> {
public:
  FMP(RTT::TaskContext* parent, const std::string& name = "FMP", const std::string& port_name = "fmp", const std::string& description = "FMP sensor data")
    : RawSensor<fmp_msgs::RawData, fmp_msgs::SensorData>(parent, name, port_name, description)
    , voltage("Voltage")
    , temp("Temperature")
    , alpha("alpha")
    , beta("beta")
    , vTAS("TAS")
    , pStat("pStat")
    , pDyn("pDyn")
    , aileron("Aileron")
    , elevator("Elevator")
    , rudder("Rudder")
    , elevatorTrim("ElevatorTrim")
    , wMot("Motor")
    , humi("Humidity")
  {
    properties()->addProperty(voltage);
    properties()->addProperty(temp);
    properties()->addProperty(humi);
    properties()->addProperty(alpha);
    properties()->addProperty(beta);
    properties()->addProperty(vTAS);
    properties()->addProperty(pStat);
    properties()->addProperty(pDyn);
    properties()->addProperty(aileron);
    properties()->addProperty(elevator);
    properties()->addProperty(rudder);
    properties()->addProperty(elevatorTrim);
    properties()->addProperty(wMot);
  }

  virtual bool convert(const fmp_msgs::RawData& raw) {
    static const double nan = std::numeric_limits<double>::quiet_NaN();

    data.voltage      = raw.voltage != 0xFFFF      ? voltage.convert(raw.voltage)           : nan;
    data.temp         = raw.temp != 0xFFFF         ? temp.convert(raw.temp)                 : nan;
    data.humi         = raw.humi != 0xFFFF         ? humi.convert(raw.humi)                 : nan;
    data.alpha        = raw.alpha != 0xFFFF        ? alpha.convert(raw.alpha)               : nan;
    data.beta         = raw.beta != 0xFFFF         ? beta.convert(raw.beta)                 : nan;
    data.vTAS         = raw.vTAS != 0xFFFF         ? vTAS.convert(raw.vTAS)                 : nan;
    data.pStat        = raw.pStat != 0xFFFF        ? pStat.convert(raw.pStat)               : nan;
    data.pDyn         = raw.pDyn != 0xFFFF         ? pDyn.convert(raw.pDyn)                 : nan;
    data.aileron      = raw.aileron != 0xFFFF      ? aileron.convert(raw.aileron)           : nan;
    data.elevator     = raw.elevator != 0xFFFF     ? elevator.convert(raw.elevator)         : nan;
    data.rudder       = raw.rudder != 0xFFFF       ? rudder.convert(raw.rudder)             : nan;
    data.elevatorTrim = raw.elevatorTrim != 0xFFFF ? elevatorTrim.convert(raw.elevatorTrim) : nan;
    data.wMot         = raw.wMot != 0xFFFF         ? wMot.convert(raw.wMot)                 : nan;
    updated();
    return true;
  }

protected:
  LinearTransformation<int,double> voltage, temp, alpha, beta, vTAS, pStat, pDyn, aileron, elevator, rudder, elevatorTrim, wMot;
  QuadraticTransformation<int,double> humi;
};

} // namespace Sensors
} // namespace uxvcos

#endif // UXVCOS_SENSORS_FMP_H
