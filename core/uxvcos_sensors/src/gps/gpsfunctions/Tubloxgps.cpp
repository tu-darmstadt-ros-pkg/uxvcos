#include "Tubloxgps.h"
#include <rtt/Logger.hpp>

TUBLOXGPS::TUBLOXGPS(System::BaseSerialPort *gpsPort)
{
    GPSIncomeBufferLoad  = 0;
    GPSSBASDGPS.DGPS_Age = 9999;
    
    //--> Uebergebe TUBLOXGPS-Objekt Zugriff auf Schnittstelle
    this->gpsPort = gpsPort;
}

TUBLOXGPS::~TUBLOXGPS()
{
}

signed char TUBLOXGPS::WaitForAckNck(uint8_t pClassID, uint8_t pMsgID, unsigned short pTimeout)
{
    pTimeout = 10;

    while (pTimeout-- > 0)
    {
        switch (ProcessGPSQueue())
        {
        case uBloxNakData:
            if ((pClassID == AckNakRaw.ClassID) && (pMsgID == AckNakRaw.MsgID))
                return -1;
            break;
        case uBloxAckData:
            if ((pClassID == AckAckRaw.ClassID) && (pMsgID == AckAckRaw.MsgID))
                return 1;
            break;
        default:
            break;
        }
    }
    return 0;
}

bool TUBLOXGPS::Initialize()
{
    unsigned char ii;
    signed char OK = FALSE;

    GPSIncomeBufferLoad = 0;
    
    if (!gpsPort) return false;

    if (!gpsPort->setBaudrate(9600)) {
      RTT::log(RTT::Error) << "Could not set GPS baudrate" << RTT::endlog();
      return false;
    }
    if (!gpsPort->open()) {
      RTT::log(RTT::Error) << "Could not open GPS port" << RTT::endlog();
      return false;
    }

    for (ii=0; ii < 5; ii++)
    {
        // set GPS: Port 1, out UBX, 57600 8N1, autobauding off
        // class 0x06, ID 0x00
	    gpsPort->send(UBXCFGPRT_PRT_57600,sizeof(UBXCFGPRT_PRT_57600));
	
        OK = WaitForAckNck(0x06, 0x00, 500);
	
        if (OK) break;
    }
    
    gpsPort->close();
    gpsPort->setBaudrate(38400);
    gpsPort->open();
    
    OK = FALSE;

    for (ii=0; ii < 5; ii++)
    {
        // set GPS: Port 1, out UBX, 57600 8N1, autobauding off
        // class 0x06, ID 0x00
        gpsPort->send(UBXCFGPRT_PRT_57600,sizeof(UBXCFGPRT_PRT_57600));
  
        OK = WaitForAckNck(0x06, 0x00, 500);
  
        if (OK) break;
    }
    gpsPort->close();
    gpsPort->setBaudrate(57600);
    gpsPort->open();
    
    OK = FALSE;

    for (ii=0; ii < 5; ii++)
    {
        // set GPS: Port 1, out UBX, 57600 8N1, autobauding off
        // class 0x06, ID 0x00
        gpsPort->send(UBXCFGPRT_PRT_57600,sizeof(UBXCFGPRT_PRT_57600));
	
        OK = WaitForAckNck(0x06, 0x00, 500);
	
        if (OK) break;
    }

    if (OK)
    {
        // set GPS: Nav-Solution Port 1&2
        // class 0x06, ID 0x01
            gpsPort->send(UBXCFGMSG_NAV_SOL,sizeof(UBXCFGMSG_NAV_SOL));
	
        OK = WaitForAckNck(0x06, 0x01, 500);
    }

    if (OK)
    {
        // set GPS: SatInfo on Port 1, every 4th measurement
        // class 0x06, ID 0x01
            gpsPort->send(UBXCFGMSG_NAV_SVINFO4,sizeof(UBXCFGMSG_NAV_SVINFO4));
	
        OK = WaitForAckNck(0x06, 0x01, 500);
    }

    if (OK)
    {
        // set GPS: Measurement rate 250ms, Navigation rate 4 Hz, UTC align
        // class 0x06, ID 0x08
	    gpsPort->send(UBXCFGRATE4HZ,sizeof(UBXCFGRATE4HZ));
	    
        OK = WaitForAckNck(0x06, 0x08, 500);
    }

    if (OK)
    {
        // set GPS: SBAS enabled, (Rng,Cor,Int), max. 3, autoscan
        // class 0x06, ID 0x16
	    gpsPort->send(UBXCFGSBASAUTOSCAN,sizeof(UBXCFGSBASAUTOSCAN));
	
        OK = WaitForAckNck(0x06, 0x16, 500);
    }

    if (OK)
    {
        // set GPS: DGPSInfo on Port 1
        // class 0x06, ID 0x01
            gpsPort->send(UBXCFGMSG_NAV_DGPS,sizeof(UBXCFGMSG_NAV_DGPS));
	
        OK = WaitForAckNck(0x06, 0x01, 500);
    }

    if (OK)
    {
        // set GPS: SBASInfo on Port 1
        // class 0x06, ID 0x01
            gpsPort->send(UBXCFGMSG_NAV_SBAS09,sizeof(UBXCFGMSG_NAV_SBAS09));
            
        OK = WaitForAckNck(0x06, 0x01, 500);
    }

    return (OK == TRUE);
}

UBLOX_DATA_TYPE TUBLOXGPS::ProcessGPSQueue(void)
{
    unsigned char  uBloxClass;                      	// Decoded uBlox class
    unsigned char  uBloxId;                         	// Decoded uBlox Id
    unsigned short uBloxLength;                     	// Total Length of uBlox message
    unsigned short i;                               	// Counter
    unsigned char  NCH;                             	// Number of satellite informations
    unsigned char  ok=FALSE;                        	// Decoding flag
    UBLOX_DATA_TYPE packettyp = uBloxNoDataAvailable;
    signed char DecodeRes;
    
    if (!gpsPort) return uBloxNoDataAvailable;

    //--> Lade Daten vom GPS-Port in den GPSIncomeBuffer
    //-----------------------------------------------------------------------------------------------------------------------------
    int receivedLoad = gpsPort->receive(GPSIncomeBuffer + GPSIncomeBufferLoad, sizeof(GPSIncomeBuffer)-GPSIncomeBufferLoad);
    if (receivedLoad == -1)
      return uBloxNoDataAvailable; //--> Fehler beim Aufruf von receive
    else
      GPSIncomeBufferLoad += (unsigned short) receivedLoad;
    //-----------------------------------------------------------------------------------------------------------------------------

    NavSolRaw_updated = FALSE;
    SvInfoRaw_updated = FALSE;
    AckNakRaw_updated = FALSE;
    AckAckRaw_updated = FALSE;

    // if there is nothing just quit
    if (GPSIncomeBufferLoad < 6 + 2)
        return uBloxNoDataAvailable;

    // search for the first appearance of a valid uBlox header
    for (i=0; i<GPSIncomeBufferLoad-1; i++)
    {
        if (GPSIncomeBuffer[i] == UBLOX_COM_SYNC1)
            if (GPSIncomeBuffer[i+1] == UBLOX_COM_SYNC2)
                break;
    }

    // delete everything at the beginning of the buffer
    // that is not a uBlox header
    if (i > 0)
    {
	  memcpy(GPSIncomeBuffer, (GPSIncomeBuffer+i) , GPSIncomeBufferLoad-i);
	  GPSIncomeBufferLoad -= i;
    }

    // Decode GPS data
    DecodeRes = Ubx.CheckForMsg(GPSIncomeBuffer, GPSIncomeBufferLoad,
                                UBLOX_COM_SYNC1, UBLOX_COM_SYNC2,
                                &uBloxClass, &uBloxId, &uBloxLength);
    if (DecodeRes == TRUE)
    {
        ok = FALSE;
        packettyp = uBloxUnknownData;

        // uBlox, GPS-Nav, Class 0x01, ID 0x06
        if ((uBloxClass == UBLOX_NAVSOL_CLASS) && (uBloxId == UBLOX_NAVSOL_ID) && (uBloxLength == sizeof(UBLOX_NAVSOL)))
        {
            // Processing
            memcpy(&NavSolRaw, GPSIncomeBuffer+6, sizeof(NavSolRaw));

            GPSRaw.ITOW = NavSolRaw.ITOW;
            GPSRaw.Frac = NavSolRaw.Frac;
            GPSRaw.week = NavSolRaw.week;
            GPSRaw.GPSfix = NavSolRaw.GPSfix;
            GPSRaw.Flags = NavSolRaw.Flags;
            GPSRaw.ECEF_X = NavSolRaw.ECEF_X;
            GPSRaw.ECEF_Y = NavSolRaw.ECEF_Y;
            GPSRaw.ECEF_Z = NavSolRaw.ECEF_Z;
            GPSRaw.Pacc = NavSolRaw.Pacc;
            GPSRaw.ECEFVX = NavSolRaw.ECEFVX;
            GPSRaw.ECEFVY = NavSolRaw.ECEFVY;
            GPSRaw.ECEFVZ = NavSolRaw.ECEFVZ;
            GPSRaw.SAcc = NavSolRaw.SAcc;
            GPSRaw.PDOP = NavSolRaw.PDOP;
            GPSRaw.res1 = NavSolRaw.res1;
            GPSRaw.numSV = NavSolRaw.numSV;
            GPSRaw.res2 = NavSolRaw.res2;

            ProcessNavSol(&GPSRaw);
            NavSolRaw_updated = TRUE;
            ok = TRUE;
            packettyp = uBloxNavSolData;
        }

        // uBlox, GPS-Nav-SvInfo, Class 0x01, ID 0x30
        if ((uBloxClass == UBLOX_NAV_SVINFO_CLASS) && (uBloxId == UBLOX_NAV_SVINFO_ID))
        {
            // header
            NCH = GPSIncomeBuffer[10];
            if (uBloxLength == 8+NCH*12)
            {
                // Processing
                if (NCH >= 16) NCH = 16;
                memcpy(&SVInfoRaw, GPSIncomeBuffer+6, 8+NCH*12);
                ProcessGPSSvInfo();
                SvInfoRaw_updated = TRUE;
                ok = TRUE;
                packettyp = uBloxSvInfoData;
            }
        }

        // uBlox, DGPSInfo, Class 0x01, ID 0x31
        if ((uBloxClass == UBLOX_NAV_DGPS_CLASS) && (uBloxId == UBLOX_NAV_DGPS_ID))
        {
            // just use the high level info, drop the info for each satellite
            if (uBloxLength >= 8+16)
            {
                // Processing
                memcpy(&GPSDGPSInfo, GPSIncomeBuffer+6, sizeof(GPSDGPSInfo));
                ProcessDGPSInfo();
                DGPSRaw_updated = TRUE;
                ok = TRUE;
                packettyp = uBloxDGPSData;
            }
        }

        // uBlox, SBASInfo, Class 0x01, ID 0x32
        if ((uBloxClass == UBLOX_NAV_SBAS_CLASS) && (uBloxId == UBLOX_NAV_SBAS_ID))
        {
            // just use the high level info, drop the info for each satellite
            if (uBloxLength >= 8+12)
            {
                // Processing
                memcpy(&GPSSBASInfo, GPSIncomeBuffer+6, sizeof(GPSSBASInfo));
                ProcessSBASInfo();
                SBASRaw_updated = TRUE;
                ok = TRUE;
                packettyp = uBloxSBASData;
            }
        }

        // uBlox, NACK, Class 0x05, ID 0x00
        if ((uBloxClass == UBLOX_ACK_CLASS) && (uBloxId == UBLOX_NACK_ID) && (uBloxLength == sizeof(UBLOX_ACKNAK)))
        {
            // Processing
            memcpy(&AckNakRaw, GPSIncomeBuffer+6, sizeof(UBLOX_ACKNAK));
            AckNakRaw_updated = TRUE;
            ok = TRUE;
            packettyp = uBloxNakData;
        }

        // uBlox, ACK, Class 0x05, ID 0x01
        if ((uBloxClass == UBLOX_ACK_CLASS) && (uBloxId == UBLOX_ACK_ID) && (uBloxLength == sizeof(UBLOX_ACKACK)))
        {
            // Processing
            memcpy(&AckAckRaw, GPSIncomeBuffer+6, sizeof(UBLOX_ACKACK));
            AckAckRaw_updated = TRUE;
            ok = TRUE;
            packettyp = uBloxAckData;
        }

        // delete uBlox packet (decoded or not)
        memcpy(GPSIncomeBuffer, (GPSIncomeBuffer+uBloxLength+8), GPSIncomeBufferLoad-uBloxLength-8);
        GPSIncomeBufferLoad -= uBloxLength + 8;
	    
        return packettyp;
    }

    // if there was a checksum error or if the income buffer is fully loaded and no message was decoded delete this message
    if ((DecodeRes == -1) || (GPSIncomeBufferLoad == GPSINCOMEBUFFERSIZE))
        GPSIncomeBuffer[0] = 0;

    return packettyp;
}

void TUBLOXGPS::ProcessNavSol(const Data::Navigation::RawGPS* rawGPS)
{
    GPSNav.utc = (rawGPS->ITOW % 86400000)/(double)1000.0;
    GPSNav.signalQuality = rawGPS->GPSfix;
    
    if (GPSNav.signalQuality != 0)
    {
        GpsCalculations.ECEF2LLA((double)rawGPS->ECEF_X/(double)100.0,
                                 (double)rawGPS->ECEF_Y/(double)100.0,
                                 (double)rawGPS->ECEF_Z/(double)100.0,
                                 &GPSNav.lat,					// [rad] at this point
				 &GPSNav.lon,					// [rad] at this point
                                 &GPSNav.altitude);
        GpsCalculations.ECEF2NED(GPSNav.lat, GPSNav.lon,			// [rad] at this point
                                 (double)rawGPS->ECEFVX/(double)100.0,
                                 (double)rawGPS->ECEFVY/(double)100.0,
                                 (double)rawGPS->ECEFVZ/(double)100.0,
                                 &GPSNav.v_n,
                                 &GPSNav.v_e,
                                 &GPSNav.v_d);

        GPSNav.trueCourse = atan2(GPSNav.v_e, GPSNav.v_n);
        GPSNav.groundSpeed= sqrt(pow(GPSNav.v_n,2)+pow(GPSNav.v_e,2));
        GPSNav.numberOfSatellites = rawGPS->numSV;
        GPSNav.pdop = (float) rawGPS->PDOP / 100.0f;
    }
    else
    {
        GPSNav.lat   = 0.0;
        GPSNav.lon   = 0.0;
        GPSNav.altitude = 0.0;
        GPSNav.v_n   = 0.0;
        GPSNav.v_e   = 0.0;
        GPSNav.v_d   = 0.0;
        GPSNav.trueCourse = 0.0;
        GPSNav.groundSpeed= 0.0;
        GPSNav.numberOfSatellites = rawGPS->numSV;
        GPSNav.pdop = (float) rawGPS->PDOP / 100.0f;
    }
}

void TUBLOXGPS::ProcessGPSSvInfo(void)
{
  GPSSvInfo.itow    = SVInfoRaw.ITOW;
  GPSSvInfo.nch     = SVInfoRaw.NCH;
  GPSSvInfo.res1    = SVInfoRaw.RES1;
  GPSSvInfo.res2    = SVInfoRaw.RES2;

  for(unsigned int i = 0; i < SVInfoRaw.NCH && i < GPSSvInfo.NUMBER_OF_CHANNELS; ++i) {
    GPSSvInfo.sv[i].chn     = SVInfoRaw.sv[i].chn;
    GPSSvInfo.sv[i].svID    = SVInfoRaw.sv[i].SVID;
    GPSSvInfo.sv[i].flags   = SVInfoRaw.sv[i].Flags;
    GPSSvInfo.sv[i].qi      = SVInfoRaw.sv[i].QI;
    GPSSvInfo.sv[i].cno     = SVInfoRaw.sv[i].CNO;
    GPSSvInfo.sv[i].elev    = SVInfoRaw.sv[i].Elev;
    GPSSvInfo.sv[i].azim    = SVInfoRaw.sv[i].Azim;
    GPSSvInfo.sv[i].prrRes  = SVInfoRaw.sv[i].PRRres;
  }
}

void TUBLOXGPS::ProcessDGPSInfo(void)
{
    GPSSBASDGPS.DGPS_Age    = GPSDGPSInfo.AGE;
    GPSSBASDGPS.DGPS_BaseID = GPSDGPSInfo.BASEID;
    GPSSBASDGPS.DGPS_Status = GPSDGPSInfo.STATUS;
}

void TUBLOXGPS::ProcessSBASInfo(void)
{
    GPSSBASDGPS.SBAS_GEO     = GPSSBASInfo.GEO;
    GPSSBASDGPS.SBAS_Mode    = GPSSBASInfo.MODE;
    GPSSBASDGPS.SBAS_Sys     = GPSSBASInfo.SYS;
    GPSSBASDGPS.SBAS_Service = GPSSBASInfo.SERVICE;
}

void TUBLOXGPS::GetGPSRaw(Data::Navigation::RawGPS *pGPSRaw)
{
   *pGPSRaw = GPSRaw;
}

void TUBLOXGPS::GetNavSol(Data::Navigation::GPS *pGPSNav)
{
   *pGPSNav = GPSNav;
}

void TUBLOXGPS::GetSvInfo(Data::Navigation::SVInfo *pSvInfo)
{
   *pSvInfo = GPSSvInfo;
}

void TUBLOXGPS::GetNavDGPSSBAS(Data::Navigation::SBASDGPS *pSBASDGPS)
{
   *pSBASDGPS = GPSSBASDGPS;
}

bool TUBLOXGPS::configureNavigationEngine(enum DynModel dynModel)
{
  unsigned char payload[] = { 0x01, 0x00, dynModel, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char message[sizeof(payload) + 8];
  unsigned short length = Ubx.EncodeMsg(message, sizeof(message), 0xB5, 0x62, payload, sizeof(payload), 0x06, 0x24);
  if (!length) return false;
  return gpsPort->send(message,length) && (WaitForAckNck(0x06, 0x24, 500) == TRUE);
}

bool TUBLOXGPS::configureNavigationEngine(enum FixMode fixMode)
{
  unsigned char payload[] = { 0x04, 0x00, 0x00, fixMode, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char message[sizeof(payload) + 8];
  unsigned short length = Ubx.EncodeMsg(message, sizeof(message), 0xB5, 0x62, payload, sizeof(payload), 0x06, 0x24);
  if (!length) return false;
  return gpsPort->send(message,length) && (WaitForAckNck(0x06, 0x24, 500) == TRUE);
}
