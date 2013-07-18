//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
