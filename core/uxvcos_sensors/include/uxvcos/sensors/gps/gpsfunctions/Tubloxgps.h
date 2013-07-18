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

#ifndef UXVCOS_SENSOR_GPS_TUBLOXGPS_H
#define UXVCOS_SENSOR_GPS_TUBLOXGPS_H

#ifdef __GNUC__
 #ifndef GCC3X_PACK8
  #define GCC3X_PACK8 __attribute__((__packed__))
 #endif
#else
 #ifndef GCC3X_PACK8
  #define GCC3X_PACK8
  #pragma pack(push,1)
 #endif
#endif

#include <string.h>
#include <stdint.h>
#include <types/navigation.h>
#include <system/BaseSerialPort.h>

#include "ubx-protocol.h"
#include "gpscalculations.h"

#define GPSINCOMEBUFFERSIZE     1000

#define UBLOX_NAVSOL_CLASS      0x01
#define UBLOX_NAVSOL_ID         0x06
#define UBLOX_NAV_SVINFO_CLASS  0x01
#define UBLOX_NAV_SVINFO_ID     0x30
#define UBLOX_NAV_DGPS_CLASS    0x01
#define UBLOX_NAV_DGPS_ID       0x31
#define UBLOX_NAV_SBAS_CLASS    0x01
#define UBLOX_NAV_SBAS_ID       0x32

#define UBLOX_ACK_CLASS         0x05
#define UBLOX_NACK_ID           0x00
#define UBLOX_ACK_ID            0x01

#define UBLOX_COM_SYNC1         0xB5      		//!< first protocol sync byte uBlox GPS
#define UBLOX_COM_SYNC2         0x62      		//!< second protocol sync byte uBlox GPS

//! \brief Port 1, out UBX, 9600 8N1, autobauding off
static const uint8_t UBXCFGPRT_PRT_9600[] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        	// Port 1
    0x80,0x25,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0x9E,0x99
};

//! \brief Port 1, out UBX, 57600 8N1, autobauding off
static const uint8_t UBXCFGPRT_PRT_57600[] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,
    0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,        	// Port 1
    0x00,0xE1,0x00,0x00,0x05,0x00,0x01,0x00,
    0x00,0x00,0x00,0x00,
    0xDA,0xAD
};

//! \brief Nav-Solution Port 1&2
static const uint8_t UBXCFGMSG_NAV_SOL[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x06,0x00,0x01,0x01,0x00,
    0x16,0x9E
};

//! \brief SatInfo on Port 1, every 4th measurement
static const uint8_t UBXCFGMSG_NAV_SVINFO4[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x30,0x00,0x04,0x00,0x00,
    0x42,0x77
};

//! \brief SBASInfo on Port 1, every 2th measurement
static const uint8_t UBXCFGMSG_NAV_SBAS02[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x32,0x00,0x02,0x00,0x00,
    0x42,0x7B
};

//! \brief SBASInfo on Port 1, every 9th measurement
static const uint8_t UBXCFGMSG_NAV_SBAS09[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x32,0x00,0x09,0x00,0x00,
    0x49,0x90
};

//! \brief DGPSInfo on Port 1
static const uint8_t UBXCFGMSG_NAV_DGPS[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x31,0x00,0x02,0x00,0x00,
    0x41,0x76
};

//! \brief Position Solution in ECEF, every measurement
static const uint8_t UBXCFGMSG_NAV_POSECEF[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x01,0x00,0x01,0x00,0x00,
    0x10,0x83
};

//! \brief Velocity Solution in ECEF, every measurement
static const uint8_t UBXCFGMSG_NAV_VELECEF[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x11,0x00,0x01,0x00,0x00,
    0x20,0xD3
};

//! \brief Clock Solution, every measurement
static const uint8_t UBXCFGMSG_NAV_CLOCK[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x22,0x00,0x01,0x00,0x00,
    0x31,0x28
};

//! \brief Raw Measurement Data on Port 1, every measurement
static const uint8_t UBXCFGMSG_RXM_RAW[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x02,0x10,0x00,0x01,0x00,0x00,
    0x20,0xD4
};

//! \brief GPS Constellation Almanach Data, every measurement
static const uint8_t UBXCFGMSG_RXM_ALM[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,0x02,0x30,0x00,0x01,0x00,0x00,0x40,0x74
};

//! \brief Subframe Buffer, every measurement
static const uint8_t UBXCFGMSG_RXM_SRFB[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,0x02,0x11,0x00,0x01,0x00,0x00,0x21,0xD9
};

//! \brief GPS Constellation Ephemeris Data, every measurement
static const uint8_t UBXCFGMSG_RXM_EPH[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,0x02,0x30,0x00,0x01,0x00,0x00,0x40,0x74
};

//! \brief DGPSInfo on Port 1, every 5th measurement
static const uint8_t UBXCFGMSGNAVDGPS05[] = {
    0xB5,0x62,0x06,0x01,0x06,0x00,
    0x01,0x31,0x00,0x05,0x00,0x00,
    0x44,0x7F
};

//! \brief SBAS enabled, (Rng,Cor,Int), max. 3, autoscan
static const uint8_t UBXCFGSBASAUTOSCAN[] = {
    0xB5,0x62,0x06,0x16,0x08,0x00,
    0x01,0x07,0x03,0x00,0x00,0x00,0x00,0x00,
    0x2F,0xD5
};

//! \brief Measurement rate 250ms, Navigation rate 4 Hz, UTC align
static const uint8_t UBXCFGRATE4HZ[] = {
    0xB5,0x62,0x06,0x08,0x06,0x00,
    0xFA,0x00,0x01,0x00,0x01,0x00,
    0x10,0x96
};

//! \brief Ublox data type identifier
typedef enum _UBLOX_DATA_TYPE
{
    uBloxNoDataAvailable = -2,                      //!<
    uBloxUnknownData     = -1,                      //!<
    uBloxNakData         = 0,                       //!<
    uBloxAckData         = 1,                       //!<
    uBloxNavSolData      = 2,                       //!<
    uBloxSvInfoData      = 3,                       //!<
    uBloxDGPSData        = 4,                       //!<
    uBloxSBASData        = 5                        //!<
}UBLOX_DATA_TYPE;

//! ubx gps frame NAV-SOL (0x01 0x06 )
// Navigation Solution Information  This message combining Position, velocity and time solution in ECEF
typedef struct GCC3X_PACK8 UBLOX_NAVSOL
{
    uint32_t     ITOW;                          //! [ms] GPS Millisecond Time of Week
    int32_t       Frac;                          //! [ns] Nanoseconds remainder of rounded ms above, range -500000 .. 500000
    int16_t     week;                          //! GPS week (GPS time)
    uint8_t    GPSfix;                        //! details see 'GpsFix' constants
    uint8_t    Flags;                         //! details see 'GpsFlags' constants
    int32_t       ECEF_X;                        //! [cm]  ECEF X coordinate
    int32_t       ECEF_Y;                        //! [cm]  ECEF Y coordinate
    int32_t       ECEF_Z;                        //! [cm]  ECEF Z coordinate
    uint32_t     Pacc;                          //! [cm]  3D Position Accuracy Estimate
    int32_t       ECEFVX;                        //! [cm/s]  ECEF X velocity
    int32_t       ECEFVY;                        //! [cm/s]  ECEF Y velocity
    int32_t       ECEFVZ;                        //! [cm/s]  ECEF Z velocity
    uint32_t     SAcc;                          //! [cm/s] Speed Accuracy Estimate
    uint16_t   PDOP;                          //! PDOP  -  Position DOP
    uint8_t    res1;                          //! Reserved
    uint8_t    numSV;                         //! Number of SVs used in Nav Solution
    uint32_t     res2;                          //! Reserved
}UBLOX_NAVSOL;

//! space vehicle data
typedef struct GCC3X_PACK8 UBLOX_SVDATA
{
  uint8_t  chn;                             //!< channel number, range 0..NCH-1
  uint8_t  SVID;                            //!< Satellite ID
  uint8_t  Flags;                           //!< bitmask see 'SvFlags'
  int8_t  QI;                              //!< constants see 'SvQualityInicator'
  uint8_t  CNO;                             //!< [dbHz ] Carrier to Noise Ratio (Signal Strength)
  int8_t  Elev;                            //!< [deg] Elevation in integer degrees
  int16_t Azim;                            //!< [deg] Azimuth in integer degrees
  int32_t   PRRres;                          //!< [cm]  Pseudo range residual in centimetres
}UBLOX_SVDATA;

//! ubx satellite frame NAV-SVINFO (0x01 0x30)
typedef struct GCC3X_PACK8 UBLOX_SVINFO
{
  uint32_t    ITOW;                           //! [ms]  GPS Millisecond time of week
  uint8_t   NCH;                            //! Number of channels range 0..16
  uint8_t   RES1;                           //! Reserved
  uint16_t  RES2;                           //! Reserved
  UBLOX_SVDATA    sv[16];
}UBLOX_SVINFO;

//! ubx satellite frame NAV-DGPS (0x01 0x31)
typedef struct GCC3X_PACK8 UBLOX_DGPS
{
    uint32_t    ITOW;                           //! [ms]  GPS Millisecond time of week
    int32_t    AGE;                            //! [ms] Age of newest correction data
    int16_t  BASEID;                         //! DGPS Base station ID
    int16_t  BASEHLTH;                       //! DGPS Base station health status
    uint8_t   NCH;                            //! Number of channels for which correction data is following (is skipped in this app)
    uint8_t   STATUS;                         //! DGPS correction type status
    //!  00 - none
    //!  01 - PR+PRR correction
    //!  10 - PR+PRR+CP correction
    //!  11 - High accuracy PR+PRR+CP correction
    uint16_t   RES;                           //! Reserved
}UBLOX_DGPS;

//! ubx satellite frame NAV-SBAS (0x01 0x32)
typedef struct GCC3X_PACK8 UBLOX_SBAS
{
    uint32_t    ITOW;                           //! [ms]  GPS Millisecond time of week
    uint8_t   GEO;                            //! PRN number of the GEO where correction and integrity is used from
    uint8_t   MODE;                           //! SBAS mode
    //! 0 - disabled
    //! 1 - Enabled integrity
    //! 3 - Enabled test mode
    int8_t   SYS;                            //! SBAS System
    //! -1 - Unknown
    //!  0 - WAAS
    //!  1 - EGNOS
    //!  2 - MSAS
    //!  16- GPS
    uint8_t   SERVICE;                        //! SBAS service available (flags)
    //!  bit0 - Ranging
    //!  bit1 - Corrections
    //!  bit2 - Integrity
    //!  bit3 - Testmode
    uint8_t   CNT;                            //! Number of SV data following (is skipped in this app)
    uint8_t   RES[3];                         //! Reserved
}UBLOX_SBAS;

//! ubx satellite frame ACK-NAK (0x05 0x00)
typedef struct GCC3X_PACK8 UBLOX_ACKNAK
{
    uint8_t   ClassID;                        //!
    uint8_t   MsgID;                          //!
}UBLOX_ACKNAK;

//! ubx satellite frame ACK-ACK (0x05 0x01)
typedef struct GCC3X_PACK8 UBLOX_ACKACK
{
    uint8_t   ClassID;                        //!
    uint8_t   MsgID;                          //!
}UBLOX_ACKACK;


//! uBlox GPS device processing Class
//!
//! * Resets and configures an uBlox GPS device
//! * It gets incomming GPS data from an ISR and stores it.
//! * Check GPSIncomeBuffer for a diagramm ready to be processed
//! * Decode NavSol, SVInfo and SBASDGPS diagramms
//!
//! Info:
//! blocking call!
//! maximum clock is 500kHz
//! there is a periode of 60ms after powering up where no interface activity should take place
class TUBLOXGPS
{
public:

    TUBLOXGPS(System::BaseSerialPort *gpsPort);
    virtual ~TUBLOXGPS(void);

    //! Initialise the GPS send queue and configure the GPS device
    //! blocking call!
    //!
    //! \param *pGPS_send_queue a pointer to the GPS send queue
    //! \return true if ok
    bool Initialize();

    //! check GPSIncomeBuffer for an complete uBlox diagramm
    //! copy found diagramms to local variables
    //!
    //! \return uBlox diagramm type
    UBLOX_DATA_TYPE ProcessGPSQueue(void);

    //! Get GPS raw data from last datagram
    //! retrieves a copy in *pGPSRaw
    //!
    void GetGPSRaw(Data::Navigation::RawGPS *pGPSRaw);

    //! Process received NavSol diagramm and copy result to the
    //! given variable (pointer)
    //!
    //! \param *pGPSNav processed GPS data
    void GetNavSol(Data::Navigation::GPS *pGPSNav);

    //! Process received SVInfo diagramm and copy result to the
    //! given variable (pointer)
    //!
    //! \param *pSvInfo processed SVInfo data
    void GetSvInfo(Data::Navigation::SVInfo *pSvInfo);

    //! Process received NavDGPSSBAS diagramm and copy result to the
    //! given variable (pointer)
    //!
    //! \param *pSBASDGPS processed SBAS&DGPS data
    void GetNavDGPSSBAS(Data::Navigation::SBASDGPS *pSBASDGPS);

    void ProcessNavSol(const Data::Navigation::RawGPS *rawGPS = 0);

    enum DynModel { Portable = 0, Stationary = 2, Pedestrian = 3, Automotive = 4, Sea = 5, Airborne1g = 6, Airborne2g = 7, Airborne4g };
    bool configureNavigationEngine(enum DynModel dynModel);

    enum FixMode { Only2D = 1, Only3D = 2, Auto2D3D = 3 };
    bool configureNavigationEngine(enum FixMode fixMode);

private:
    signed char WaitForAckNck(uint8_t pClassID, uint8_t pMsgID, unsigned short timeout);
    void ProcessGPSSvInfo(void);
    void ProcessDGPSInfo(void);
    void ProcessSBASInfo(void);
    
    uint8_t     	        GPSIncomeBuffer[GPSINCOMEBUFFERSIZE];
    unsigned short    		GPSIncomeBufferLoad;
    TUBX_PROTOCOL     		Ubx;
    TGpsCalculations  		GpsCalculations;

    unsigned char     		NavSolRaw_updated;
    unsigned char     		SvInfoRaw_updated;
    unsigned char     		DGPSRaw_updated;
    unsigned char     		SBASRaw_updated;
    unsigned char     		AckNakRaw_updated;
    unsigned char     		AckAckRaw_updated;
    UBLOX_NAVSOL      		NavSolRaw;
    UBLOX_SVINFO            SVInfoRaw;
    UBLOX_DGPS        		GPSDGPSInfo;
    UBLOX_SBAS        		GPSSBASInfo;
    UBLOX_ACKNAK      		AckNakRaw;
    UBLOX_ACKACK      		AckAckRaw;
    
    Data::Navigation::RawGPS    GPSRaw;
    Data::Navigation::GPS       GPSNav;
    Data::Navigation::SVInfo    GPSSvInfo;
    Data::Navigation::SBASDGPS  GPSSBASDGPS;
    
    System::BaseSerialPort 	*gpsPort;
};

#endif // UXVCOS_SENSOR_GPS_TUBLOXGPS_H
