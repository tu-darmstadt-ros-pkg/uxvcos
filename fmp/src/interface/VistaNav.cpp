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

#include "VistaNav.h"

#include <rtt/Logger.hpp>
#include <types/interface.h>
#include <types/fmp.h>

static const unsigned char STARTSEQUENCE[] = { 0x4D,0x43,0x01,0xD1,0x00,0x00,0xD2,0x77 };
static const unsigned char STOPSEQUENCE[] =  { 0x4D,0x43,0x01,0xD2,0x00,0x00,0xD3,0x7A };

/*
4D4306D10000D78B --> GetFirmwareVersion
4D4306D20000D88E --> GetSerialNumber
4D4306D30000D991 --> GetCalibrationInfo
4D4301D10000D277 --> StartNav
4D4301D20000D37A --> StopNav
*/

namespace Interface {

VistaNav::VistaNav(const std::string& name)
  : SerialInterface(name)
  , rawGPS("RawGPS")
  , imu(this)
  , baro(this)
{
  this->ports()->addPort( rawGPS );
  // dataPool->getNewDataOnPortEvent()->connect(boost::bind(&VistaNav::newData, this, _1));
}

VistaNav::~VistaNav()
{}

bool VistaNav::startHook() {
  if (!SerialInterface::startHook()) return false;

  if (!serialPort->send(STARTSEQUENCE, sizeof(STARTSEQUENCE))) return false;

  RTT::log( RTT::Info ) << "Successfully started VistaNav interface" << RTT::endlog();
  return true;
}

void VistaNav::stopHook() {
  serialPort->send(STOPSEQUENCE, sizeof(STOPSEQUENCE));
  SerialInterface::stopHook();
}

void VistaNav::updateHook() {
  SerialInterface::updateHook();
}

bool VistaNav::newData(const Data::Streamable* data) {
  const Data::FMP::FMPIMUData* imuData = dynamic_cast<const Data::FMP::FMPIMUData*>(data);
  if (imuData) {
    Data::Interface::RawIMU output;

    output.RAccel[0] = imuData->RAccelX;
    output.RAccel[1] = imuData->RAccelY;
    output.RAccel[2] = imuData->RAccelZ;
    output.RGyro[0]  = imuData->RGyroX;
    output.RGyro[1]  = imuData->RGyroY;
    output.RGyro[2]  = imuData->RGyroZ;

    output.setTimestamp(*imuData);
    imu.Set( output );
  }

  if (imuData) {
    Data::Interface::RawBaro output;

    output.value = imuData->RBaro;

    output.setTimestamp(*imuData);
    baro.Set( output );
  }

  const Data::FMP::FMPGPSNavSol* gpsData = dynamic_cast<const Data::FMP::FMPGPSNavSol*>(data);
  if (gpsData) {
    Data::Navigation::RawGPS output;

    output.ITOW = gpsData->ITOW;
    output.Frac = gpsData->Frac;
    output.week = gpsData->week;
    output.GPSfix = gpsData->GPSfix;
    output.Flags = gpsData->Flags;
    output.ECEF_X = gpsData->ECEF_X;
    output.ECEF_Y = gpsData->ECEF_Y;
    output.ECEF_Z = gpsData->ECEF_Z;
    output.Pacc = gpsData->Pacc;
    output.ECEFVX = gpsData->ECEFVX;
    output.ECEFVY = gpsData->ECEFVY;
    output.ECEFVZ = gpsData->ECEFVZ;
    output.SAcc = gpsData->SAcc;
    output.PDOP = gpsData->PDOP;
    output.res1 = gpsData->res1;
    output.numSV = gpsData->numSV;
    output.res2 = gpsData->res2;

    output.setTimestamp(*gpsData);
    rawGPS.write( output );
  }

  if (imuData) return true;
  return false;
}

} // namespace Interface

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( Interface::VistaNav )
