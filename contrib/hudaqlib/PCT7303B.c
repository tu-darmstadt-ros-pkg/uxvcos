/****************************************************************/
/**@file PCT7303B.c:
 * Description: API layer for TEDIA PCT-7303B card.             *
 * Dependency: Windows 32, Windows 64 or Linux                  *
 *                Copyright 2010 Jaroslav Fojtik                *
 ****************************************************************/

#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
#else
  #include <sys/io.h>
#endif
#include <malloc.h>
#include <math.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"


#define DI_CHANNELS   1		///< Amount of Digital Outputs
#define DO_CHANNELS   1		///< Amount of Digital Inputs
#define DA_CHANNELS   0		///< Amount of Analog Outputs
#define AD_CHANNELS   0		///< Amount of Analog Inputs
#define ENC_CHANNELS  3		///< Amount of Encoders
#define CTR_CHANNELS  1		///< Amount of counters
#define STEP_CHANNELS 0		///< Amount of steppers


#define MASTERFREQUENCY 1000	///< Internal oscillator frequency is 1kHz

#define IrcResetCounter 2	///< Binary flag for IrcOptions


/** Cache of PCT-7303B state */
typedef struct
{
  UserDataHeader Hdr;                   ///< General device setup

  int EncOptions[ENC_CHANNELS];		///< Bit options for encoders

  unsigned __int8 DOReg;		///< Port configuration register
  unsigned __int8 IRQCfgReg;
  unsigned __int8 INTEnReg;
  unsigned __int8 TimerReg;
  unsigned __int8 CNTEnReg;
} PCT_7303B_Private;


/** Symbolic aliases for available registers inside BADR1 read. */
typedef enum
{
  DINReg     = 0,		// RD only
  DOUTReg    = 4,		// WR only

  IRQCfgReg  = 0x180,		// WR only
  IRQStatusReg = 0x180,		// RD only
  IRQClrReg =  0x184,		// WR only
  
  TimerReg =  0x3F0,		// RW
  INTEnReg =  0x18C,		// WR only

  CNT0SetReg = 0x200,		// RW
  CNT1SetReg = 0x204,		// RW
  CNT2SetReg = 0x208,		// RW

  CNTEnReg = 0x380,		// WR only
  CNTCtrReg= 0x384,		// WR only

} MEM_BAR1;	// Memory space #0 on F1


/** write to memory mapped device byte wise, bytes are stored on every 4th position. */
static __inline void StoreByte(size_t Ptr, int Offset, unsigned __int8 value)
{
  ((volatile unsigned __int8 *)(Ptr))[Offset] = value;
}

static __inline void StoreDword(size_t Ptr, int Offset, unsigned __int32 value)
{
  ((volatile unsigned __int32 *)(Ptr))[Offset] = value;
}


/** write to memory mapped device byte wise through cache. */
static __inline void StoreCachedByte(size_t Ptr, int Offset, unsigned __int8 value, unsigned __int8 *CachedValue)
{
  if (*CachedValue != value)
    {
    *CachedValue = value;
    StoreByte(Ptr,Offset,value);
    }
}


/** This inline is used for reading tightly packed OX9162 local configuration registers. */
static __inline unsigned __int8 GetBytePlain(size_t Ptr, int Offset)
{
  return ((volatile unsigned __int8 *)(Ptr))[Offset];
}


/****************************************************************
 *                                                              *
 *                 DIGITAL INPUTS & OUTPUTS                     *
 *                                                              *
 ****************************************************************/


/** Get data from digital input. */
static int PCT7303DIRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=DI_CHANNELS) return HUDAQBADARG;
  return GetBytePlain(DevRecord->DrvRes.Resources.MemResources[0].Base,DINReg);
}


/** Write data to digital output. */
static HUDAQSTATUS PCT7303DOWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned value)
{
  if (channel>=DO_CHANNELS) return HUDAQBADARG;
  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, DOUTReg, value,
                  &((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->DOReg);
  return HUDAQSUCCESS;
}


/** Write one bit to digital output. */
static void PCT7303DOWriteBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;
  if (bit>=8) return;   //there are only 8 bits available in PCT7303

  DoutByte = (((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->DOReg);
  if (value) DoutByte |= (1<<bit);
       else DoutByte &= ~(1<<bit);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, DOUTReg, DoutByte,
                  & ((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->DOReg);
}


static void PCT7303DOWriteMultipleBits(const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;

  DoutByte = (((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->DOReg);
  DoutByte = (DoutByte & ~mask) | (value & mask);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, DOUTReg, DoutByte,
                  &((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->DOReg);
}


static double PCT7303DIOGetParameter(unsigned channel, HudaqParameter param)
{
  switch((int)param)
    {
    case HudaqDINUMBITS:
           return (channel<DI_CHANNELS) ? 8 : WRONG_VALUE;
    case HudaqDONUMBITS:
           return (channel<DO_CHANNELS) ? 8 : WRONG_VALUE;
    case HudaqDINUMCHANNELS:
	   return DI_CHANNELS;
    case HudaqDONUMCHANNELS:
	   return DO_CHANNELS;
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS PCT7303DIOSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, int value)
{

  switch((int)param)
    {
    case HudaqDINUMBITS:
           if(channel>=DI_CHANNELS) return WRONG_VALUE;
           return (value==8) ? 0 : WRONG_VALUE;
    case HudaqDONUMBITS:
           if(channel>=DI_CHANNELS) return WRONG_VALUE;
           return (value==8) ? 0 : WRONG_VALUE;
    case HudaqDINUMCHANNELS:
	   return DI_CHANNELS;
    case HudaqDONUMCHANNELS:
	   return DO_CHANNELS;
    case HudaqDOMODE:
          if(channel>=DO_CHANNELS) return WRONG_VALUE;
          return HUDAQSUCCESS;
    }

  return WRONG_VALUE;
}


/****************************************************************
 *                                                              *
 *                          ENCODERS                            *
 *                                                              *
 ****************************************************************/

static double PCT7303BEncGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
//size_t Ptr1;
PCT_7303B_Private *Cache;

  Cache = (PCT_7303B_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return WRONG_VALUE;

  switch(param)
    {
    case HudaqEncRESETONREAD:
           return(Cache->EncOptions[channel] & IrcResetCounter);

/*
    case HudaqEncMODE:
           pCTRi = (IRC_Control *)&(((char *)&(Cache->EncCache))[channel]);
           return(pCTRi->IRC_MODE);
    case HudaqEncCOUNTCONTROL:
           pCTRi = (IRC_Control *)&(((char *)&(Cache->EncCache))[channel]);
           return(pCTRi->BLOCK_MODE);
    case HudaqEncRESETMODE:
           pCTRi = (IRC_Control *)&(((char *)&(Cache->EncCache))[channel]);
           return(pCTRi->RESET_MODE);
    case HudaqEncI:
           *(__int32 *)IrcValue = GetDword(DevRecord->DrvRes.Resources.MemResources[0].Base, IRCSTATUS);
           return(IrcValue[channel]);
*/
    case HudaqEncNUMCHANNELS:
           return ENC_CHANNELS;
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS PCT7303BEncSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
size_t Ptr1;
PCT_7303B_Private *Cache;

  Cache = (PCT_7303B_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return HUDAQBADARG;
  Ptr1 = DevRecord->DrvRes.Resources.MemResources[0].Base;

  switch(param)
    {
    case HudaqEncRESETONREAD:
           if(value!=0)
             Cache->EncOptions[channel] |= IrcResetCounter;
           else
             Cache->EncOptions[channel] &= ~IrcResetCounter;
           return HUDAQSUCCESS;

/*
    case HudaqEncMODE:
           if (value<0 || value>=4) return HUDAQBADARG;
           *(unsigned __int32 *)&IrcValue = Cache->EncCache;
           pCTRi = (IRC_Control *)&IrcValue[channel];

           pCTRi->IRC_MODE = (int)value;
           StoreCachedDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue, &Cache->EncCache);
           return HUDAQSUCCESS;

    case HudaqEncRESETMODE:
	   if (value<0 || value>=7) return HUDAQBADARG;
           *(unsigned __int32 *)&IrcValue = Cache->EncCache;
           pCTRi = (IRC_Control *)&IrcValue[channel];

           pCTRi->RESET_MODE = (int)value;
           StoreCachedDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue, &Cache->EncCache);
           return HUDAQSUCCESS;

    case HudaqEncCOUNTCONTROL:
	   if (value<0 || value>=4) return HUDAQBADARG;
           *(unsigned __int32 *)&IrcValue = Cache->EncCache;
           pCTRi = (IRC_Control *)&IrcValue[channel];

           pCTRi->BLOCK_MODE = (int)value;
           StoreCachedDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue, &Cache->EncCache);
           return HUDAQSUCCESS;
*/
    }

  return HUDAQNOTSUPPORTED;
}


/* static HUDAQSTATUS PCT7303BEncReset(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr2;
MF624_Private *Cache;
__int8 IrcValue[ENC_CHANNELS];
IRC_Control *pCTRi;
char OldResetMode;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return HUDAQBADARG;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[0].Base;

  *(unsigned __int32*)&IrcValue = Cache->EncCache;
  pCTRi = (IRC_Control *)&IrcValue[channel];

  OldResetMode = pCTRi->RESET_MODE; //store original reset mode
  pCTRi->RESET_MODE = 1;                               //reset encoder
  StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
  pCTRi->RESET_MODE = OldResetMode;                    //enable counting
  StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
  return HUDAQSUCCESS;
} */


static int PCT7303BEncRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr1;
__int32 RetVal;
PCT_7303B_Private *Cache;

  if (channel>=ENC_CHANNELS) return 0;
  Cache = (PCT_7303B_Private *)DevRecord->DrvRes.DriverData;
  Ptr1 = DevRecord->DrvRes.Resources.MemResources[0].Base;

	// read a value from encoder
  StoreByte(Ptr1, CNTCtrReg, 1<<channel);	
  RetVal = GetBytePlain(Ptr1,CNT0SetReg) + 
           0x100 * GetBytePlain(Ptr1,CNT1SetReg) +
	   0x10000 * GetBytePlain(Ptr1,CNT2SetReg);

  if(Cache->EncOptions[channel] & IrcResetCounter) //RESET counter on every read
    {
    StoreByte(Ptr1, CNT0SetReg, 0);
    StoreByte(Ptr1, CNT1SetReg, 0);
    StoreByte(Ptr1, CNT2SetReg, 0);
    StoreByte(Ptr1, CNTCtrReg, 0x10<<channel);
    }

  return RetVal;
}


/****************************************************************
 *                                                              *
 *                COUNTERS PWM + Count + Step                   *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS PCT7303BPWMWrite(const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle)
{
double T;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;

  if (frequency<=0) return HUDAQBADARG; //Allow duty cycle 0 and 1 for f=0.

  T = MASTERFREQUENCY/frequency;
  if(T<1) 
  {
    StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, TimerReg, 1,
                  & ((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
    return HUDAQPARTIAL;
  }
  if(T>255)
  {
    StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, TimerReg, 255,
                  & ((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
    return HUDAQPARTIAL;
  }

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, TimerReg, 
		  (unsigned __int8)T,
                  &((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);

  return HUDAQSUCCESS;
}


static HUDAQSTATUS PCT7303BCtrReset(const DeviceRecord *DevRecord, unsigned channel)
{
  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, TimerReg,
		  0, &((PCT_7303B_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
  return HUDAQSUCCESS;
}


static int PCT7303BCtrRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=CTR_CHANNELS) return 0;
  return GetBytePlain(DevRecord->DrvRes.Resources.MemResources[0].Base, TimerReg);
return 0;
}


/********************************************************************/

/* Initialization procedure for PCT-7303B card. */
static int InitPCT7303(DeviceRecord *DevRecord, int IniOptions)
{
PCT_7303B_Private *Cache;
size_t Base;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<2) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(PCT_7303B_Private)) return -4;

  DevRecord->pCT = &CTPCT7303;

  if ((IniOptions & HudaqOpenNOINIT)==0)
  {                           //initialize hardware only if no other application is running
    Base = DevRecord->DrvRes.Resources.MemResources[0].Base;

	/* Disable interrupts */
    Cache->IRQCfgReg = 0;
    StoreByte(Base, IRQCfgReg, 0);
    Cache->INTEnReg = 0;		/* Clear interrupt logic. */
    StoreByte(Base, INTEnReg, 0);

	/* Initialization of DI and DO. */
    Cache->DOReg = 0;
    StoreByte(Base, DOUTReg, 0);

	/* Initialization of internal timer/counter. */
    Cache->TimerReg = 0;
    StoreByte(Base, TimerReg, 0);	/* Stop timer. */

	/* Initialization of encoders. */
    Cache->CNTEnReg = 0;
    StoreByte(Base, CNTEnReg, 0);	/* Stop encoders. */
  }

return 1; //success
}


/** Internal cleanup procedure for PCT7303B */
static void DonePCT7303(DeviceRecord *DevRecord)
{
  if (DevRecord==NULL) return;

  DevRecord->pCT = &CtDummy;
}



/****************************************************************
 *                                                              *
 *                    GET/SET PARAMETERS                        *
 *                                                              *
 ****************************************************************/


static HUDAQSTATUS PCT7303IRQSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, int value)
{
size_t Base;

  Base = DevRecord->DrvRes.Resources.MemResources[0].Base;

/*
  switch((int)param)
    {
    case HudaqIRQ+0:
	{
	 if(value)
	 {
	   StoreCachedByte(Base, IRQCfgReg, 1, &((PCT_7303B_Private *)DevRecord->DrvRes.DriverData)->IRQCfgReg);
	   StoreByte(Base, IRQClrReg, 0xFF);
	   StoreCachedByte(Base, INTEnReg, 0x80, &((PCT_7303B_Private *)DevRecord->DrvRes.DriverData)->INTEnReg);
         }
         else
	 { 
	   StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, INTEnReg, 0,
			   &((PCT_7303B_Private *)DevRecord->DrvRes.DriverData)->INTEnReg);
	   StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[0].Base, IRQCfgReg, 0,
			   &((PCT_7303B_Private *)DevRecord->DrvRes.DriverData)->IRQCfgReg);
         }
        return HUDAQSUCCESS;
        }
    case HudaqIRQ+1:
           return HUDAQSUCCESS;
    }
*/

  return WRONG_VALUE;
}


static double PCT7303IRQGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
  switch((int)param)
    {
    case HudaqIRQ+0: 
	   return ((PCT_7303B_Private *)DevRecord->DrvRes.DriverData)->Hdr.IRQcounter;
    }
  return WRONG_VALUE;
}


static HUDAQSTATUS PCT7303SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCT7303DIOSetParameter(DevRecord,channel,param,(int)value);
//  case HudaqAI:  
//  case HudaqAO:  
  case HudaqEnc: return PCT7303BEncSetParameter(DevRecord,channel,param,(int)value);
//  case HudaqPWM:
//  case HudaqCtr: 
//  case HudaqStep:
  case HudaqIRQ: return PCT7303IRQSetParameter(DevRecord,channel,param,(int)value);
  }
return HUDAQNOTSUPPORTED;
}

static double PCT7303GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCT7303DIOGetParameter(channel,param);
//case HudaqAI:  
//case HudaqAO:  
  case HudaqEnc: return PCT7303BEncGetParameter(DevRecord,channel,param);
//case HudaqPWM:
//case HudaqCtr:
//case HudaqStep:
  case HudaqIRQ: return PCT7303IRQGetParameter(DevRecord,channel,param);
  }
return WRONG_VALUE;
}


const CallTable CTPCT7303 =
    {
    "PCT7303B", 0x1760, 0x0201,
    InitPCT7303,
    DonePCT7303,

    PCT7303SetParameter,
    PCT7303GetParameter,
    NULL,

        // INITIALIZE DI callers
    PCT7303DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    PCT7303DOWrite,
    PCT7303DOWriteBit,
    PCT7303DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    NULL,
    NULL,
        // INITIALIZE AO callers
    NULL,		                       // Not available
    NULL,
        // INITIALIZE Enc callers
    PCT7303BEncRead,
    NULL,
        // INITIALIZE Ctr callers
    PCT7303BCtrRead,
    PCT7303BCtrReset,
        // INITIALIZE PWM callers
    PCT7303BPWMWrite,            
    NULL,			// Not available
    NULL,
        // INITIALIZE Step callers
    NULL,                       // Not available
    };
    