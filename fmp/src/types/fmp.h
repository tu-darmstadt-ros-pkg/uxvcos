#ifndef DATA_TYPES_FMP_H
#define DATA_TYPES_FMP_H

#include <data/Typekit.h>
#include <data/Streamable.h>

namespace Data {
namespace FMP {

struct FMPRawData : public Streamable
{
  uint16_t voltage;			// Voltage of the PCBs input			[mV]
  uint16_t temp;			// Temperature-measurement of the SHT75 [14 bit ticks]
  uint16_t humi;			// Humidity-measurment of the SHT75		[12 bit ticks]
  uint16_t alpha;			// flight-log alpha						[mdeg]
  uint16_t beta;			// flight-log beta						[mdeg]
  uint16_t vTAS;			// flight-log vTAS						[Hz]
  uint16_t pStat;			// static pressure						[mbar]
  uint16_t pDyn;			// dynamic pressure						[mbar]
  uint16_t aileron;			// Encoder Aileron						[mdeg]
  uint16_t elevator;		        // Encoder Elevator						[mdeg]
  uint16_t rudder;			// Encoder Rudder						[mdeg]
  uint16_t elevatorTrim;                // Encoder Elevator Assy				[mdeg]
  uint16_t wMot;			// revolutions motor					[rpm]

  FMPRawData()
    : voltage(0)
    , temp(0)
    , humi(0)
    , alpha(0)
    , beta(0)
    , vTAS(0)
    , pStat(0)
    , pDyn(0)
    , aileron(0)
    , elevator(0)
    , rudder(0)
    , elevatorTrim(0)
    , wMot(0)
  {}


  InStream& operator<<(InStream& in)
  {
    in
      >> voltage
      >> temp
      >> humi
      >> alpha
      >> beta
      >> vTAS
      >> pStat
      >> pDyn
      >> aileron
      >> elevator
      >> rudder
      >> elevatorTrim
      >> wMot;

    return in;
  };

  OutStream& operator>>(OutStream& out) const
  {
    out
      << voltage
      << temp
      << humi
      << alpha
      << beta
      << vTAS
      << pStat
      << pDyn
      << aileron
      << elevator
      << rudder
      << elevatorTrim
      << wMot;
      
    return out;
  };

  const char* const* getFieldNames() const
  {
          static const char* const FieldNames[] = {"voltage", "temp", "humi", "alpha", "beta", "vTAS",
                  "pStat", "pDyn", "aileron", "elevator", "rudder", "elevatorTrim", "wMot", 0 };
          return FieldNames;
  }

  const char* getShortName() const
  {
          return "FmpRawData";
  }

  virtual size_t getSize() const
  { return sizeof(*this); }
};

struct FMPSensorData : public Streamable
{
  double voltage;         // Voltage of the PCBs input            [V]
  double temp;            // Temperature-measurement of the SHT75 [°C]
  double humi;            // Humidity-measurement of the SHT75     [°C]
  double alpha;           // flight-log alpha                     [deg]
  double beta;            // flight-log beta                      [deg]
  double vTAS;            // flight-log vTAS                      [m/s]
  double pStat;           // static pressure                      [hPa]
  double pDyn;            // dynamic pressure                     [Pa]
  double aileron;         // Encoder Aileron                      [deg]
  double elevator;        // Encoder Elevator                     [deg]
  double rudder;          // Encoder Rudder                       [deg]
  double elevatorTrim;    // Encoder Elevator Assy                [deg]
  double wMot;            // revolutions motor                    [rpm]

  FMPSensorData()
    : voltage(0)
    , temp(0)
    , humi(0)
    , alpha(0)
    , beta(0)
    , vTAS(0)
    , pStat(0)
    , pDyn(0)
    , aileron(0)
    , elevator(0)
    , rudder(0)
    , elevatorTrim(0)
    , wMot(0)
  {}

  InStream& operator<<(InStream& in)
  {
    in
      >> voltage
      >> temp
      >> humi
      >> alpha
      >> beta
      >> vTAS
      >> pStat
      >> pDyn
      >> aileron
      >> elevator
      >> rudder
      >> elevatorTrim
      >> wMot;

    return in;
  };

  OutStream& operator>>(OutStream& out) const
  {
    out
      << voltage
      << temp
      << humi
      << alpha
      << beta
      << vTAS
      << pStat
      << pDyn
      << aileron
      << elevator
      << rudder
      << elevatorTrim
      << wMot;

    return out;
  };

  const char* const* getFieldNames() const
  {
      static const char* const FieldNames[] = {"voltage", "temp", "humi", "alpha", "beta", "vTAS",
          "pStat", "pDyn", "aileron", "elevator", "rudder", "elevatorTrim", "wMot", 0 };
      return FieldNames;

  }

  const char* getShortName() const
  {
      return "FmpSensorData";
  }

  virtual size_t getSize() const
  { return sizeof(*this); }
};

};
};

UXVCOS_DECLARE_TYPEKIT(FMP);
UXVCOS_DECLARE_TYPE(Data::FMP::FMPRawData);
UXVCOS_DECLARE_TYPE(Data::FMP::FMPSensorData);

#endif // DATA_TYPES_FMP_H
