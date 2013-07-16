#ifndef __BAROMETER_H__
#define __BAROMETER_H__

#include <math.h>

class TBAROMETER
{

public:
  TBAROMETER(void);
  virtual ~TBAROMETER(void);

  unsigned char SetMeasurement(double pPressure);
  void SetQNH(double pQNH);

  double GetPressure(void);
  double GetAltitudeWithQNH(void);
  double GetQNH(void);

private:
  double QNH;                              //!< Pressure for getting the airport height
  double Pressure;                         //!< Static air pressure [mBar]
};

#endif
