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
