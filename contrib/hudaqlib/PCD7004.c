/****************************************************************/
/**@file PCD7004.c:
 * Description: API layer for TEDIA PCD-7004 card.              *
 * Dependency: Windows 32, Windows 64 or Linux                  *
 *                Copyright 2009-2010 Jaroslav Fojtik           *
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


#define DI_CHANNELS   4		///< Amount of Digital Outputs
#define DO_CHANNELS   4		///< Amount of Digital Inputs
#define DA_CHANNELS   0         ///< Amount of Analog Outputs
#define AD_CHANNELS   0         ///< Amount of Analog Inputs
#define ENC_CHANNELS  0         ///< Amount of Encoders
#define CTR_CHANNELS  1         ///< Amount of counters
#define STEP_CHANNELS 0         ///< Amount of steppers


#define MASTERFREQUENCY 1000        ///< Internal oscillator frequency is 1kHz


/** Cache of PCD-7004 state */
typedef struct
{
  UserDataHeader Hdr;                   ///< General device setup

  unsigned __int8 DIOReg[4];		///< Port configuration register
  unsigned __int8 DIOCfgReg;
  unsigned __int8 IRQCfgReg;
  unsigned __int8 INTEnReg;
  unsigned __int8 TimerReg;
} PCD_7004_Private;


/** Symbolic aliases for available registers inside BADR4 read. */
typedef enum
{
  DIOReg0    = 0,
  DIOReg1    = 1,	//4
  DIOReg2    = 2,	//8,
  DIOReg3    = 3,	//0xC,

  DIOCfgReg  = 0x80/4,

  IRQCfgReg  = 0x200/4,		// WR only
  IRQStatusReg = 0x200/4,	// RD only
  IRQClrReg =  0x204/4,		// WR only
  
  TimerReg =  0x208/4,		// RW
  INTEnReg =  0x20C/4,

} MEM4IO;

static __int8 Ports[DI_CHANNELS] = {DIOReg0, DIOReg1, DIOReg2, DIOReg3};


/** write to memory mapped device byte wise, bytes are stored on every 4th position. */
static __inline void StoreByte(size_t Ptr, int Offset, unsigned __int8 value)
{
  ((volatile unsigned __int8 *)(Ptr))[4*Offset] = value;
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
  return ((volatile unsigned __int8 *)(Ptr))[4*Offset];
}


/****************************************************************
 *                                                              *
 *                 DIGITAL INPUTS & OUTPUTS                     *
 *                                                              *
 ****************************************************************/


/** Get data from digital input. */
static int PCD7004DIRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=DI_CHANNELS) return HUDAQBADARG;
  return GetBytePlain(DevRecord->DrvRes.Resources.MemResources[1].Base,Ports[channel]);
}


/** Write data to digital output. */
static HUDAQSTATUS PCD7004DOWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned value)
{
  if (channel>=DO_CHANNELS) return HUDAQBADARG;
  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, Ports[channel], value,
                  &((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg[channel] );
  return HUDAQSUCCESS;
}


static HUDAQSTATUS PCD7004DIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, unsigned *values)
{
unsigned FlagX = 0;
unsigned i;
size_t Base;
unsigned __int8 Cache[DI_CHANNELS+1];  //allocate more bytes for padding ind

  if (number==0 || channels==NULL || values==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return HUDAQFAILURE;

  Base = DevRecord->DrvRes.Resources.MemResources[1].Base;

	/* Schedule what to read. */
  for(i=0; i<number; i++)
  {
    if (channels[i]<DI_CHANNELS)
      {
      FlagX |= 1<<(channels[i]);
      }
    else
      {
      values[i] = 0;
      FlagX |= 0x8000;
      }
    }
    
    /* Read isolated DIs */
  for(i=0; i<DI_CHANNELS; i++)
    {
    if(FlagX & (1<<i))
      {
      Cache[i] = GetBytePlain(Base,Ports[i]);
      }
    }  
    
	/* Final store of numbers read. */
  for(i=0; i<number; i++)
  {
    if(channels[i]<DI_CHANNELS)
      {
      values[i] = Cache[channels[i]];
      }
  }      
  
  if(FlagX == 0x8000) return HUDAQFAILURE;	//no valid value returned
  if(FlagX & 0x8000) return HUDAQPARTIAL;	//several values are valid and several are not
  return HUDAQSUCCESS;			//all values are valid (or no channel asked)
}


static HUDAQSTATUS PCD7004DOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, const unsigned *values)
{
unsigned FlagX = 0;
unsigned i;
size_t Base;
unsigned __int8 *Cache;

  if (number==0 || channels==NULL || values==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return HUDAQFAILURE;

  Base = DevRecord->DrvRes.Resources.MemResources[1].Base;
  Cache = ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg;

    /* schedule what to write */
  for(i=0; i<number; i++)
  {
    if (channels[i]<DI_CHANNELS)
      {
      if(Cache[i] != values[i])
        {
        FlagX |= 1<<(channels[i]);
        Cache[i] = values[i];
        }
      }
    else
      {
      FlagX |= 0x8000;
      }
    }

    /* Final store of DOs */
  for(i=0; i<DI_CHANNELS; i++)
    {
    if(FlagX & (1<<i))
      {
      StoreByte(Base, Ports[i], Cache[i]);
      }
    }  
      
  if(FlagX == 0x8000) return HUDAQFAILURE;	//no valid value returned
  if(FlagX & 0x8000) return HUDAQPARTIAL;	//several values are valid and several are not
  return HUDAQSUCCESS;			//all values are valid (or no channel asked)
}


/** Write one bit to digital output. */
static void PCD7004DOWriteBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;
  if (bit>=8) return;		/* There are only 8 bits available in PCD7004. */

  DoutByte = (((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg[channel]);
  if (value) DoutByte |= (1<<bit);
       else DoutByte &= ~(1<<bit);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, Ports[channel], DoutByte,
                  & ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg[channel]);
}


static void PCD7004DOWriteMultipleBits(const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;

  DoutByte = (((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg[channel]);
  DoutByte = (DoutByte & ~mask) | (value & mask);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, Ports[channel], DoutByte,
                  & ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOReg[channel] );
}


static double PCD7004DIOGetParameter(unsigned channel, HudaqParameter param)
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


static HUDAQSTATUS PCD7004DIOSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, int value)
{
unsigned __int8 chval;

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
          chval = ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOCfgReg;
	  //printf("chval =  %d\n",chval);          
     
          switch(channel)
          {
            case 0: if(value==1) chval &= ~0x1;
        	    if(value==0) chval |= 0x1;
        	    break;
            case 1: if(value==1) chval &= ~0x02;
        	    if(value==0) chval |= 0x02;
        	    break;
            case 2: if(value==1) chval &= ~0x04;
        	    if(value==0) chval |= 0x04;
		    break;
            case 3: if(value==1) chval &= ~0x08;
        	    if(value==0) chval |= 0x08;
        	    break;
          }
          StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, DIOCfgReg, chval,
                  & ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->DIOCfgReg );
          return HUDAQSUCCESS;
    }

  return WRONG_VALUE;
}


/****************************************************************
 *                                                              *
 *                COUNTERS PWM + Count + Step                   *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS PCD7004PWMWrite(const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle)
{
double T;
HUDAQSTATUS ret = HUDAQSUCCESS;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;

  if (frequency<=0) return HUDAQBADARG; //Allow duty cycle 0 and 1 for f=0.

  T = MASTERFREQUENCY/frequency;
  if(T<1) 
  {
    StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, TimerReg, 1,
                  & ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
    return HUDAQPARTIAL;
  }
  if(T>255)
  {
    StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, TimerReg, 255,
                  & ((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
    return HUDAQPARTIAL;
  }

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, TimerReg, 
		  (unsigned __int8)T,
                  &((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);

  return HUDAQSUCCESS;
}


static HUDAQSTATUS PCD7004CtrReset(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=CTR_CHANNELS) return HUDAQBADARG;
  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, TimerReg, 
		  0, &((PCD_7004_Private *)(DevRecord->DrvRes.DriverData))->TimerReg);
  return HUDAQSUCCESS;
}


static int PCD7004CtrRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=CTR_CHANNELS) return 0;
  return GetBytePlain(DevRecord->DrvRes.Resources.MemResources[1].Base, TimerReg);
}





/********************************************************************/

/* Initialization procedure for PCD-7004 card. */
static int InitPCD7004(DeviceRecord *DevRecord, int IniOptions)
{
PCD_7004_Private *Cache;
int i;
size_t Base;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<2) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(PCD_7004_Private)) return -4;

  DevRecord->pCT = &CTPCD7004;

  if ((IniOptions & HudaqOpenNOINIT)==0)
  {                           //initialize hardware only if no other application is running
    Base = DevRecord->DrvRes.Resources.MemResources[1].Base;

	/* disable interrupts */
    Cache->INTEnReg = 0;
    StoreByte(Base, INTEnReg, 0);

      /* Initialization of DI and DO. */
    for(i=0; i<4; i++)
    {
      Cache->DIOReg[i] = 0;
      StoreByte(Base, Ports[i], 0);
    }

    Cache->DIOCfgReg = 0;
    StoreByte(Base, DIOCfgReg, 0);

    Cache->IRQCfgReg = 0;
    StoreByte(Base, IRQCfgReg, 0);

    Cache->TimerReg = 0;
    StoreByte(Base, TimerReg, 0);	/* Stop timer. */
  }

return 1; //success
}


/** Internal cleanup procedure for PCD7004 */
static void DonePCD7004(DeviceRecord *DevRecord)
{
  if (DevRecord==NULL) return;

  DevRecord->pCT = &CtDummy;
}



/****************************************************************
 *                                                              *
 *                    GET/SET PARAMETERS                        *
 *                                                              *
 ****************************************************************/


static HUDAQSTATUS PCD7004IRQSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, int value)
{
size_t Base;

  Base = DevRecord->DrvRes.Resources.MemResources[1].Base;

  switch((int)param)
    {
    case HudaqIRQ+0:
	{
	 if(value)
	 {
	   StoreCachedByte(Base, IRQCfgReg, 1, &((PCD_7004_Private *)DevRecord->DrvRes.DriverData)->IRQCfgReg);
	   StoreByte(Base, IRQClrReg, 0xFF);
	   StoreCachedByte(Base, INTEnReg, 0x80, &((PCD_7004_Private *)DevRecord->DrvRes.DriverData)->INTEnReg);
         }
         else
	 { 
	   StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, INTEnReg, 0,
			   &((PCD_7004_Private *)DevRecord->DrvRes.DriverData)->INTEnReg);
	   StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, IRQCfgReg, 0,
			   &((PCD_7004_Private *)DevRecord->DrvRes.DriverData)->IRQCfgReg);
         }
        return HUDAQSUCCESS;
        }
    case HudaqIRQ+1:
           return HUDAQSUCCESS;
    }

  return WRONG_VALUE;
}


static double PCD7004IRQGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
  switch((int)param)
    {
    case HudaqIRQ+0: 
	   return ((PCD_7004_Private *)DevRecord->DrvRes.DriverData)->Hdr.IRQcounter;
    case HudaqIRQ+1:
           return GetBytePlain(DevRecord->DrvRes.Resources.MemResources[1].Base,IRQStatusReg);
    }
  return WRONG_VALUE;
}


static HUDAQSTATUS PCD7004SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCD7004DIOSetParameter(DevRecord,channel,param,(int)value);
//  case HudaqAI:  
//  case HudaqAO:  
//  case HudaqEnc: 
//  case HudaqPWM:
//  case HudaqCtr: 
//  case HudaqStep:
    case HudaqIRQ: return PCD7004IRQSetParameter(DevRecord,channel,param,(int)value);
  }
return HUDAQNOTSUPPORTED;
}

static double PCD7004GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCD7004DIOGetParameter(channel,param);
//case HudaqAI:  
//case HudaqAO:  
//case HudaqEnc:
//case HudaqPWM:
//case HudaqCtr:
//case HudaqStep:
  case HudaqIRQ: return PCD7004IRQGetParameter(DevRecord,channel,param);
  }
return WRONG_VALUE;
}


const CallTable CTPCD7004 =
    {
    "PCD7004", 0x1760, 0x0101,
    InitPCD7004,
    DonePCD7004,

    PCD7004SetParameter,
    PCD7004GetParameter,
    NULL,

        // INITIALIZE DI callers
    PCD7004DIRead,
    GenericDIReadBit,           //Generic implementation
    PCD7004DIReadMultiple,
        // INITIALIZE DO callers
    PCD7004DOWrite,
    PCD7004DOWriteBit,
    PCD7004DOWriteMultipleBits,
    PCD7004DOWriteMultiple,
        // INITIALIZE AI callers
    NULL,
    NULL,
        // INITIALIZE AO callers
    NULL,
    NULL,
        // INITIALIZE Enc callers
    NULL,                       // Not available
    NULL,
        // INITIALIZE Ctr callers
    PCD7004CtrRead,
    PCD7004CtrReset,
        // INITIALIZE PWM callers
    PCD7004PWMWrite,            
    NULL,			// Not available
    NULL,
        // INITIALIZE Step callers
    NULL,                       // Not available
    };
    