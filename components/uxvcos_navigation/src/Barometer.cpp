#include "Barometer.h"

TBAROMETER::TBAROMETER()
{
  Pressure = -1.0;
  QNH      = 1013.25f;
}

TBAROMETER::~TBAROMETER()
{
}

unsigned char TBAROMETER::SetMeasurement(double pPressure)
{
  if ((pPressure >= 100.0) && (pPressure <= 1100.0))
  {
    Pressure = pPressure;
    return true;
  }
  else
    return false;
}

void TBAROMETER::SetQNH(double pQNH)
{
  QNH = pQNH;
}

double TBAROMETER::GetPressure(void)
{
  return Pressure;
}

double TBAROMETER::GetAltitudeWithQNH(void)
{
  // Internationale Höhenformel nach ISA
  if ((Pressure >= 100.0) && (Pressure <= 1100.0))
    return 44330.0 * (1.0 - pow(Pressure / QNH, 0.19));
  else
    return -1.0f;
}

double TBAROMETER::GetQNH(void)
{
  return QNH;
}
