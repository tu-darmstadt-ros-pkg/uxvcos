/****************************************************************/
/**@file PCI1753.c:
 * Description: API layer for Advantech PCI1753 card.           *
 * Dependency: Windows 32 or Linux                  		*		
 *                Copyright 2006-2009 Jaroslav Fojtik           *
 * Note: Windows 64 will not work				*
 ****************************************************************/

#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
  #include <intrin.h>
  #define outb(Port,Data) __outbyte(Port,Data)
  #define outw(Port,Data) __outword(Port,Data)
  #define inb(Port)	  __inbyte(Port)
#else
  #include <sys/io.h>
#endif
#include <malloc.h>
#include <math.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"


#define DI_CHANNELS   12	///< Amount of Digital Outputs
#define DO_CHANNELS   12	///< Amount of Digital Inputs
#define DA_CHANNELS   0         ///< Amount of Analog Outputs
#define AD_CHANNELS   0         ///< Amount of Analog Inputs
#define ENC_CHANNELS  0         ///< Amount of Encoders
#define CTR_CHANNELS  0         ///< Amount of counters
#define STEP_CHANNELS 0         ///< Amount of steppers


/** Cache of PCI1753 state */
typedef struct
{
  UserDataHeader Hdr;                   ///< General device setup

  unsigned __int8 pconfig[4];		///< Port configuration register
  __int8 InterruptControl[4];
  unsigned __int8 DoutCache[DO_CHANNELS];
  __int8 PatternMatch;
  __int8 PatternEnable;
  __int8 ChangeEnableState;
} PCI1753_Private;


/** Symbolic aliases for available registers inside BADR0 read. */
typedef enum
{
  A0         = 0,
  B0         = 1,
  C0         = 2,
  Cfg0	     = 3,
  A1         = 4,
  B1         = 5,
  C1         = 6,
  Cfg1	     = 7,
  A2         = 8,
  B2         = 9,
  C2         = 10,
  Cfg3	     = 11,
  A3         = 12,
  B3         = 13,
  C3         = 14,
  Cfg4	     = 15,
  IrqCtrR    = 16,
  PatternMatch = 20,
  PatternEnable= 24,
  ChEnableState= 28,
} IO0IN;

static __int8 Ports[DI_CHANNELS] = {A0, B0, C0, A1, B1, C1, A2, B2, C2, A3, B3, C3};


/** write to memory mapped device byte wise, bytes are stored on every 4th position. */
static __inline void StoreByte(size_t Ptr, int Offset, unsigned __int8 value)
{
  //printf("%X %X\n",Ptr+Offset, value);
  outb(value, Ptr+Offset);
}

/** write to IO device byte through cache. */
static __inline void StoreCachedByte(size_t Ptr, int Offset, unsigned __int8 value, unsigned __int8 *CachedValue)
{
  if (*CachedValue != value)
    {
    *CachedValue = value;
    StoreByte(Ptr,Offset,value);
    }
}

/** Read from memory mapped device byte wise, bytes are stored on every 4th position. */
static __inline unsigned __int8 GetByte(size_t Ptr, int Offset)
{
  return inb(Ptr+Offset);
}


/****************************************************************
 *                                                              *
 *                 DIGITAL INPUTS & OUTPUTS                     *
 *                                                              *
 ****************************************************************/


/** Get data from digital input. */
static int PCI1753DIRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=DI_CHANNELS) return HUDAQBADARG;
  return GetByte(DevRecord->DrvRes.Resources.IOResources[0].Base,Ports[channel]);
}


/** Write data to digital output. */
static HUDAQSTATUS PCI1753DOWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned value)
{
  if (channel>=DO_CHANNELS) return HUDAQBADARG;
  StoreCachedByte(DevRecord->DrvRes.Resources.IOResources[0].Base, Ports[channel], value,
                  & ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache[channel] );
  return HUDAQSUCCESS;
}


static HUDAQSTATUS PCI1753DIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, unsigned *values)
{
unsigned FlagX = 0;
unsigned i;
size_t Base;
unsigned __int8 Cache[DI_CHANNELS+1];  //allocate more bytes for padding ind

  if (number==0 || channels==NULL || values==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return HUDAQFAILURE;

  Base = DevRecord->DrvRes.Resources.IOResources[0].Base;

    /* schedule what to read */
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
      Cache[i] = GetByte(Base,Ports[i]);
      }
    }  
    
    /* Final store od numbers read */
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


static HUDAQSTATUS PCI1753DOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, const unsigned *values)
{
unsigned FlagX = 0;
unsigned i,j;
size_t Base;
unsigned __int8 *Cache;

  if (number==0 || channels==NULL || values==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return HUDAQFAILURE;

  Base = DevRecord->DrvRes.Resources.IOResources[0].Base;
  Cache = ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache;

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

    /* Write DOs couples */
  for(i=0; i<12; i=i+3)
  {      
    j = 3 << i;
    if((FlagX & j) == j)
      {
      outw(*(__int16 *)&(Cache[i]), Base+Ports[i]);
      FlagX &= ~j;
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
static void PCI1753DOWriteBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;	/* HUDAQBADARG */
  if (bit>=8) return;   //there are only 8 bits available in each channel in PCI1753

  DoutByte = (((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache[channel]);
  if (value) DoutByte |= (1<<bit);
       else DoutByte &= ~(1<<bit);

  StoreCachedByte(DevRecord->DrvRes.Resources.IOResources[0].Base, Ports[channel], DoutByte,
                  & ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache[channel] );
}


static void PCI1753DOWriteMultipleBits(const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value)
{
__int8 DoutByte;

  if (channel>=DO_CHANNELS) return;

  DoutByte = (((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache[channel]);
  DoutByte = (DoutByte & ~mask) | (value & mask);

  StoreCachedByte(DevRecord->DrvRes.Resources.IOResources[0].Base, Ports[channel], DoutByte,
                  & ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->DoutCache[channel] );
}


static double PCI1753DIOGetParameter(unsigned channel, HudaqParameter param)
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


static HUDAQSTATUS PCI1753DIOSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, int value)
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
          chval = ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->pconfig[channel/3];
	  //printf("chval =  %d\n",chval);          
          switch(channel%3)
          {
            case 0: if(value==1) chval &= ~0x10;
        	    if(value==0) chval |= 0x10;
        	    break;
            case 1: if(value==1) chval &= ~0x02;
        	    if(value==0) chval |= 0x02;
        	    break;
            case 2: if(value==1) chval &= ~0x0A;
        	    if(value==0) chval |= 0x0A;
        	    break;
          }
          StoreCachedByte(DevRecord->DrvRes.Resources.IOResources[0].Base, Cfg0+4*(channel/3), chval,
                  & ((PCI1753_Private *)(DevRecord->DrvRes.DriverData))->pconfig[channel/3] );
          return HUDAQSUCCESS;
    }

  return WRONG_VALUE;
}

/********************************************************************/

/* Initialization procedure for AD612 card. */
static int InitPCI1753(DeviceRecord *DevRecord, int IniOptions)
{
PCI1753_Private *Cache;
int i;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumIOResources<1) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(PCI1753_Private)) return -4;

  DevRecord->pCT = &CTPCI1753;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {                           //initialize hardware only if no other application is running
      /* Initialization of DI and DO. */
    Cache->InterruptControl[0] = 0x8A;	/* Clear ISRs */
    Cache->InterruptControl[1] = 0x80;
    Cache->InterruptControl[2] = 0x80;
    Cache->InterruptControl[3] = 0x80;
    for(i=0; i<4; i++)
      StoreByte(DevRecord->DrvRes.Resources.IOResources[0].Base, IrqCtrR+i, Cache->InterruptControl[i]);

    for(i=0; i<4; i++)
     {
     Cache->pconfig[i] = 0x1B;
     StoreByte(DevRecord->DrvRes.Resources.IOResources[0].Base, Cfg0+4*i, Cache->pconfig[i]);
     }

    Cache->PatternMatch = 0;
    StoreByte(DevRecord->DrvRes.Resources.IOResources[0].Base, PatternMatch, Cache->PatternMatch);
    Cache->PatternEnable = 0;
    StoreByte(DevRecord->DrvRes.Resources.IOResources[0].Base, PatternEnable, Cache->PatternEnable);
    Cache->ChangeEnableState = 0;
    StoreByte(DevRecord->DrvRes.Resources.IOResources[0].Base, ChEnableState, Cache->ChangeEnableState);
    }

return 1; //success
}

/** Internal cleanup procedure for PCI1753 */
static void DonePCI1753(DeviceRecord *DevRecord)
{
  if (DevRecord==NULL) return;

  DevRecord->pCT = &CtDummy;
}



/****************************************************************
 *                                                              *
 *                    GET/SET PARAMETERS                        *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS PCI1753SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCI1753DIOSetParameter(DevRecord,channel,param,value);
//  case HudaqAI:  
//  case HudaqAO:  
//  case HudaqEnc: 
//  case HudaqPWM:
//  case HudaqCtr: 
//  case HudaqStep:
  }
return HUDAQNOTSUPPORTED;
}

static double PCI1753GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return PCI1753DIOGetParameter(channel,param);
//case HudaqAI:  
//case HudaqAO:  
//case HudaqEnc:
//case HudaqPWM:
//case HudaqCtr:
//case HudaqStep:
  }
return WRONG_VALUE;
}


const CallTable CTPCI1753 =
    {
    "PCI1753", 0x13FE, 0x1753,
    InitPCI1753,
    DonePCI1753,

    PCI1753SetParameter,
    PCI1753GetParameter,
    NULL,

        // INITIALIZE DI callers
    PCI1753DIRead,
    GenericDIReadBit,           //Generic implementation
    PCI1753DIReadMultiple,
        // INITIALIZE DO callers
    PCI1753DOWrite,
    PCI1753DOWriteBit,
    PCI1753DOWriteMultipleBits,
    PCI1753DOWriteMultiple,
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
    NULL,                       // Not available
    NULL,
        // INITIALIZE PWM callers
    NULL,                       // Not available
    NULL,
    NULL,
        // INITIALIZE Step callers
    NULL,                       // Not available
    };
    