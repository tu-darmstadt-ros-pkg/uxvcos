/****************************************************************/
/**@file MF614.c:
 * Description: API layer for MF614 card.                       *
 * Dependency: Windows 32, Windows 64 or Linux                  *
 *                Copyright 2006-2007 Humusoft s.r.o.           *
 ****************************************************************/

#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
#endif
#include <malloc.h>
#include <math.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"


#define MASTERFREQUENCY 20000000   ///< Master frequency of MF614 is 20MHz


/** Counter control register */
typedef struct
{
  unsigned int OutputControl:3; ///< 000-Inactive, Output Low; 001-Active, high on TC; 010-TC toggled; 011-illegal
                                ///< 100-Inactive, Output High Z; 101-Active, low on TC; 110-illegal; 111-illegal
  unsigned int Direction:1;     ///< 0-Down; 1-Up
  unsigned int CountMode:1;     ///< 0-Binary; 1-BCD
  unsigned int RepeatMode:1;    ///< 0-once; 1-repeat
  unsigned int ReloadMode:1;    ///< 0-load; 1-both
  unsigned int GateMode:1;      ///< 0-off; 1-on
  unsigned int CountSource:4;   ///< 0000-TC(n-1)/F6; 0001-Source1/F7; 0010-Source2/FOUT1; 0011-Source3/FOUT2
                                ///< 0100-Source4/??; 0101-Source5/??; 0110-Gate1/??; 0111-Gate2/??
                                ///< 1000-Gate3/??; 1001-Gate4/??; 1010-Gate5/??; 1011-F1/??
                                ///< 1100-F2/??; 1101-F3/??; 1110-F4/??; 1111-F5/??
  unsigned int EdgeMode:1;      ///< 0-disabled; 1-enabled
  unsigned int GateControl:3;   ///< 00-1; 01-IN; 10-Out_n-1; 11-Out_n+1
  unsigned int CountSourceEx:1; ///< extends one bit for Count source
  unsigned int InterruptMode:1; ///< 0-pulse; 1-latch
  unsigned int InterruptPolarity:1; ///< 0-low; 1-high
} Counter614Control;

/** Configuration structure for one analog input. */
typedef struct
{
  unsigned int Range:1;         ///< 0-5V; 1-10V
  unsigned int Bipolar:1;       ///< 0 - <0;range>; 1 - <-range;+range>
  unsigned int Raw:1;           ///< 0 - volts; 1 - raw
} ADSetup;


/** Configuration structure for one analog output. */
typedef struct
{
  unsigned __int8 LoCache;
  unsigned __int8 HiCache;

  unsigned int Raw:1;           ///< 0-volts; 1-raw
} DASetup;


/** Configuration structure for one encoder. */
typedef struct
{
  unsigned __int8 Filter;

  unsigned int ResetOnRead:1;   ///< 0-no reset; 1-reset
  unsigned int Polarity:1;      ///< 0-negative polarity; 1-positive polarity
  unsigned int Index:1;         ///< 0-disable index; 1-enable index
  unsigned int Level:1;         ///< 0-edge; 1-level
  unsigned int Quadrature:2;    ///< NonQuadrature / Quadrature mode
} EncSetup;


/** This enum contains internal state info. */
typedef enum
{
  Unspecified = 0,
  PWM,                          ///< PWM is output from counter
  OUT_0,                        ///< counter is generating 0 permanently
  OUT_1,                        ///< counter is generating 1 permanently
  Counting,                     ///< counting on external signal
  StepperPWM,                   ///< Part of stepper motor that generates output
  StepperMaster,                ///< Part of stepper motors that counts pulses
  custom                        ///< counter contains externally defined value
} Internal614CtrMode;


/** One record related to counter. */
typedef struct
{
  __int16 ACache;               ///< Counter A register cache; Load register
  __int16 BCache;               ///< Counter B register cache; Hold register
  Internal614CtrMode Mode;      ///< Flag of subsystem that uses counter
  Counter614Control CTC;        ///< Counter mode regiser
  unsigned int ResetOnRead:1;   ///< 0-no reset; 1-reset
} Counter614;


/** One record related to stepper motor. */
typedef struct
{
  double Fmin;                  ///< Minimal frequency of stepper motor
  double Fmax;                  ///< Maximal frequency of stepper motor
  signed char Direction;        ///< Direction of stepping motor
  __int32 LastPosition;         ///< Stepper motor target position
  __int32 ChainedPosition;      ///< Stepper motor position
  DWORD TimeStamp;              ///< Last refresh time mark call of HudaqStepout
  double Frequency;             ///< Current speed (frequency) of stepper motor
  double Acc;                   ///< Acceleration of stepping motor in steps/s^2
} Stepper614;


/** Ranges for both Analog Inputs and Analog Outputs - only one item for MF624. */
static const HudaqRange MF614RangeAIO[4] =
        {{-10,+10},{-5,+5},{0,+10},{0,+5}};

#define MAXIMAL_STEP_INCREMENT 500

#define DA_CHANNELS   4         ///< Amount of Analog Outputs
#define AD_CHANNELS   8         ///< Amount of Analog Inputs
#define ENC_CHANNELS  4         ///< Amount of Encoders
#define CTR_CHANNELS  5         ///< Amount of counters
#define STEP_CHANNELS 2         ///< Amount of steppers



/** Cache of MF614 state */
typedef struct
{
  UserDataHeader Hdr;                   ///< General device setup

                /* Digital inputs/outputs */
  __int8 DoutCache;                     ///< digital outputs to be cached

                /* Analog inputs/outputs */

  ADSetup AIOptions[AD_CHANNELS];       ///< Options for analog inputs

  DASetup DAOptions[DA_CHANNELS];       ///< DA values to be cached

                /* Counters and encoders */
  EncSetup EncOptions[ENC_CHANNELS];    ///< Encoder (IRC counter) state register

  Counter614 counter[CTR_CHANNELS];     ///< Structure that holds state of one counter

  Stepper614 step[STEP_CHANNELS];       ///< Stepper motor table
} MF614_Private;


#define LOOP_AD_TIMEOUT 0xFFFF  ///< timeout per loop for endless AD conversion


/** Symbolic aliases for available registers inside BADR0 read. */
typedef enum
{
  AIN        = 0,
  TDPIN      = 2,
  DIN        = 0x06,
  DALATCHEN  = 0x08,
  IRC0DATARD = 0x10,
  IRC0CMDRD  = 0x11
} BADR0IN;

typedef enum
{
  DATAREAD = 16
} BADR2IN;

/** Symbolic aliases for available registers inside BADR1 write. */
typedef enum
{
  ADCTRL     = 0,
  TDP        = 2,
  TCP        = 3,
  DOUT       = 0x06,
  DA0LO      = 0x08,
  DA0HI      = 0x09,
  IRC0DATAWR = 0x10,
  IRC0CMDWR  = 0x11
} BADR0OUT;



/** MF614 specific conversion values from A/D convertor to a voltage. */
static __inline double MF614_AD2VOLT(signed __int16 x, char Gain)
{
  if(Gain & 8)          // bipolar range
    {
    if(Gain & 16)       // <-10V;+10V>
      return 10.0*((signed __int16)(x<<4))/(double)0x8000;
    else                // <-5V;+5V>
      return 5.0*((signed __int16)(x<<4))/(double)0x8000;
    }
  else                  // unipolar range
    {
    if(Gain & 16)       // <0V;+10V>
      return 10.0*((signed __int16)(x & 0xFFF))/(double)0x1000;
    else                // <0V;+5V>
      return 5.0*((signed __int16)(x & 0xFFF))/(double)0x1000;
    }
}


/** Convert voltage to a raw values for D/A convertor. */
static __inline __int16 MF614_VOLT2DA(double y)
{
  y = 0x1000*(y+10)/20;
  if (y>=0x0FFF) return(0x0FFF);
  if (y<=0) return(0x0000);
  return( (__int16)y );
}



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

/** Read from memory mapped device byte wise, bytes are stored on every 4th position.
    This is specific to OX9162 board registers. */
static __inline unsigned __int8 GetByte(size_t Ptr, int Offset)
{
  return ((volatile unsigned __int8 *)(Ptr))[4*Offset];
}


/** Packed word from two 8bit reads. */
static __inline unsigned __int16 GetPackedWord(size_t Ptr, int Offset)
{
__int16 RetVal;
  RetVal = GetByte(Ptr,Offset);
  RetVal += 256*GetByte(Ptr,Offset);
  return RetVal;
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
static int MF614DIRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel!=0) return 0;
  return GetByte(DevRecord->DrvRes.Resources.MemResources[1].Base,DIN);
}


/** Write data to digital output. */
static HUDAQSTATUS MF614DOWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned value)
{
  if (channel!=0) return HUDAQBADARG;
  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, value,
                  & ((MF614_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
  return HUDAQSUCCESS;
}


/** Write one bit to digital output. */
static void MF614DOWriteBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value)
{
__int8 DoutByte;

  if (channel!=0) return;
  if (bit>=8) return;   //there are only 8 bits available in MF624

  DoutByte = ((MF614_Private *)(DevRecord->DrvRes.DriverData))->DoutCache;
  if (value) DoutByte |= (1<<bit);
       else DoutByte &= ~(1<<bit);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, DoutByte,
                  & ((MF614_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
}


static void MF614DOWriteMultipleBits(const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value)
{
__int8 DoutByte;

  if (channel!=0) return;

  DoutByte = ((MF614_Private *)(DevRecord->DrvRes.DriverData))->DoutCache;
  DoutByte = (DoutByte & ~mask) | (value & mask);

  StoreCachedByte(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, DoutByte,
                  &((MF614_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
}


static double MF614DIOGetParameter(unsigned channel, HudaqParameter param)
{
  switch(param)
    {
    case HudaqDINUMBITS:
           return (channel==0) ? 8 : WRONG_VALUE;
    case HudaqDONUMBITS:
           return (channel==0) ? 8 : WRONG_VALUE;
    case HudaqDINUMCHANNELS:
	   return 1;
    case HudaqDONUMCHANNELS:
	   return 1;
    }

  return WRONG_VALUE;
}


/****************************************************************
 *                                                              *
 *                  ANALOG INPUTS & OUTPUTS                     *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS MF614AISetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF614_Private *Cache;

  if (channel>=AD_CHANNELS) return HUDAQBADARG;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;

  switch(param)
    {
    case HudaqAIUNITS:
	   if (value<0 || value>=2) return HUDAQBADARG;
           Cache->AIOptions[channel].Raw = (int)value;
           return HUDAQSUCCESS;

    case HudaqAIRange:
          switch((int)value)
             {
             case 0: Cache->AIOptions[channel].Bipolar = 1;
                     Cache->AIOptions[channel].Range = 1;
                     break;
             case 1: Cache->AIOptions[channel].Bipolar = 1;
                     Cache->AIOptions[channel].Range = 0;
                     break;
             case 2: Cache->AIOptions[channel].Bipolar = 0;
                     Cache->AIOptions[channel].Range = 1;
                     break;
             case 3: Cache->AIOptions[channel].Bipolar = 0;
                     Cache->AIOptions[channel].Range = 0;
                     break;
             default:return HUDAQBADARG;
             }
          return HUDAQSUCCESS;
    }

  return HUDAQNOTSUPPORTED;
}


static double MF614AIGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF614_Private *Cache;

  if (channel>=AD_CHANNELS) return WRONG_VALUE;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;

  switch(param)
    {
    case HudaqAIUNITS:
           return(Cache->AIOptions[channel].Raw);

    case HudaqAIRange:                                     // see ::MF614RangeAIO for range indices
       if(Cache->AIOptions[channel].Bipolar==1)            // bipolar ranges
         {
         if(Cache->AIOptions[channel].Range==1) return 0;  // <-10V;+10V>
         return 1;                                         // <-5V;+5V>
         }
       else                                                // unipolar ranges
         {
         if(Cache->AIOptions[channel].Range==1) return 2;  // <0V;+10V>
         return 3;                                         // <0V;+5V>
         }

    case HudaqAINUMCHANNELS:
	   return AD_CHANNELS;
    }

  return WRONG_VALUE;
}


/** Get data from analog input MF614 specific. */
static double MF614AIRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr0,Ptr2;
short ad;
unsigned TimeoutCounter;
char Gain;
ADSetup *ChanAiCache;

  if (channel>=AD_CHANNELS) return UNDEFINED_VALUE;
  ChanAiCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->AIOptions[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[0].Base;

  Gain = channel | 0x40;
  if(ChanAiCache->Range) Gain |= 16;
  if(ChanAiCache->Bipolar) Gain |= 8;
  StoreByte(Ptr0, ADCTRL, Gain);  // select channel and start conversion

  TimeoutCounter=0;

  while ( (GetBytePlain(Ptr2,DATAREAD) & 0x04) != 0 )   // wait for conversion complete
    {
    if (TimeoutCounter++>LOOP_AD_TIMEOUT)
      return UNDEFINED_VALUE;           /*timeout*/
    }

  ad = GetByte(Ptr0,AIN) + 256*GetByte(Ptr0,AIN+1);   // read the value

  if(ChanAiCache->Raw)
    return ad;
  else
    return(MF614_AD2VOLT(ad,Gain));
}


static double MF614AOGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF614_Private *Cache;

  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=DA_CHANNELS) return WRONG_VALUE;

  switch(param)
    {
    case HudaqAOUNITS:
           return(Cache->DAOptions[channel].Raw);
    case HudaqAORange:
           return(0);                   // only bipolar range -10:10V is supported
    case HudaqAONUMCHANNELS:
	   return DA_CHANNELS;
    }

  return WRONG_VALUE;
}


/** Setup for MF614 analog outputs. */
static HUDAQSTATUS MF614AOSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF614_Private *Cache;

  if (channel>=DA_CHANNELS) return HUDAQBADARG;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;

  switch(param)
    {
    case HudaqAOUNITS:
	  if (value<0 || value>=2) return HUDAQBADARG;
          Cache->DAOptions[channel].Raw = (int)value;
          return HUDAQSUCCESS;

    case HudaqAORange:
          if((int)value == 0) return HUDAQSUCCESS;   // only one range "0" -10:10V is supported
          return HUDAQBADARG;
    }
  return HUDAQNOTSUPPORTED;
}


/** Write data to analog output. */
static void MF614AOWrite(const DeviceRecord *DevRecord, unsigned channel, double value)
{
size_t Ptr0;
unsigned __int16 RawValue;
MF614_Private *Cache;


  if (channel>=DA_CHANNELS) return;
  Cache = DevRecord->DrvRes.DriverData;

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  if(Cache->DAOptions[channel].Raw)
    RawValue = (int)value;
  else
    RawValue = MF614_VOLT2DA(value);

  StoreCachedByte(Ptr0, DA0LO+2*channel, RawValue & 0xFF, &Cache->DAOptions[channel].LoCache);
  StoreCachedByte(Ptr0, DA0HI+2*channel, (RawValue>>8) & 0x0F, &Cache->DAOptions[channel].HiCache);

  RawValue=GetByte(Ptr0, DALATCHEN);               // update the output
}


/** Write multiple data to analog output synchronized. */
static HUDAQSTATUS MF614AOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const double* values)
{
size_t Ptr0;
unsigned __int16 RawValue;
MF614_Private *Cache;
int RetVal = HUDAQSUCCESS;


  if (channels==NULL || values==NULL || number<=0) return HUDAQBADARG;
  Cache = DevRecord->DrvRes.DriverData;

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  while(number-->0)
    {
    if (*channels>=DA_CHANNELS)
      RetVal = HUDAQPARTIAL;
    else
      {
      if(Cache->DAOptions[*channels].Raw)
        RawValue = (unsigned __int16)*values;
      else
        RawValue = MF614_VOLT2DA(*values);

      StoreCachedByte(Ptr0, DA0LO + 2 * *channels, RawValue & 0xFF, &Cache->DAOptions[*channels].LoCache);
      StoreCachedByte(Ptr0, DA0HI + 2 * *channels, (RawValue>>8) & 0x0F, &Cache->DAOptions[*channels].HiCache);
      }
    values++;
    channels++;
    }
  RawValue = GetByte(Ptr0, DALATCHEN);               // update the output
return RetVal;
}


/****************************************************************
 *                                                              *
 *                          ENCODERS                            *
 *                                                              *
 ****************************************************************/


static double MF614EncGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF614_Private *Cache;
EncSetup *xEncOptions;

  if (channel>=ENC_CHANNELS) return WRONG_VALUE;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;
  xEncOptions = &Cache->EncOptions[channel];

  switch(param)
    {
    case HudaqEncFILTER:
           return((xEncOptions->Filter>0) ? 1 : 0);

    case HudaqEncRESETONREAD:
           return(xEncOptions->ResetOnRead);

    case HudaqEncRESETMODE:
           if(xEncOptions->Index == 0)
              return(HudaqEncRESNONE);
           if(xEncOptions->Level == 1)
	      return(HudaqEncRESI0);

           if(xEncOptions->Polarity == 1)
              return(HudaqEncRESIRISING);
           else
              return(HudaqEncRESIFALLING);

   case HudaqEncCOUNTCONTROL:
	   if(xEncOptions->Index==1) return(HudaqEncCOUNTENABLE);
           return(HudaqEncCOUNTI1);

    case HudaqEncI:
          return(GetByte(DevRecord->DrvRes.Resources.MemResources[1].Base, IRC0CMDRD)>>7);

    case HudaqEncNUMCHANNELS:
          return ENC_CHANNELS;
    }

return WRONG_VALUE;
}


static HUDAQSTATUS MF614EncSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
size_t Ptr0;
MF614_Private *Cache;
__int8 Register;
EncSetup *xEncOptions;

  if (channel>=ENC_CHANNELS) return HUDAQBADARG;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;
  xEncOptions = &Cache->EncOptions[channel];
  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  switch(param)
    {
    case HudaqEncFILTER:
           if (value == 0)
             {
             if(xEncOptions->Filter == 0) return HUDAQSUCCESS;
             xEncOptions->Filter = 0;
             }
           else
             {
             if(xEncOptions->Filter > 0) return HUDAQSUCCESS;
             xEncOptions->Filter = 9;             //limit to 250kHz
             }
           StoreByte(Ptr0, IRC0CMDWR+2*channel, 0x01);  // reset BP
           StoreByte(Ptr0, IRC0DATAWR+2*channel, xEncOptions->Filter); // PR0; program filter to xxx
           StoreByte(Ptr0, IRC0CMDWR+2*channel, 0x18);  // PR0 -> PSC
           return HUDAQSUCCESS;

    case HudaqEncRESETONREAD:
           xEncOptions->ResetOnRead = ((int)value==0) ? 0 : 1;
           return HUDAQSUCCESS;

    case HudaqEncRESETMODE:
           {
           switch((int)value)
             {
             case HudaqEncRESNONE:
                   xEncOptions->Index = 0;
                   break;
             case HudaqEncRESPERMANENT:
                   return HUDAQFAILURE;
             case HudaqEncRESI0:          ///< Reset Encoder when I=0
                   xEncOptions->Index = 1;
                   xEncOptions->Polarity = 0;
		   xEncOptions->Level = 1;
                   break;
	     case HudaqEncRESIRISING:
	           xEncOptions->Index = 1;
	           xEncOptions->Polarity = 1;
		   xEncOptions->Level = 0;
                   break;
             case HudaqEncRESIFALLING:
		   xEncOptions->Index = 1;
		   xEncOptions->Polarity = 0;
		   xEncOptions->Level = 0;
                   break;
             case HudaqEncRESI1:          ///< Reset Encoder when I=1
             case HudaqEncRESIEITHER:
                   return HUDAQNOTSUPPORTED; // these modes are not supported by MF614
	     default:
		   return HUDAQBADARG;
             }

           Register = 0x60;
           if(xEncOptions->Index)
             {
             StoreByte(Ptr0,IRC0CMDWR+2*channel,0x41);     // IOR 0 10 00 0 0 1
             Register|=4;
             if(xEncOptions->Polarity) Register|=2;
             if(!xEncOptions->Level) Register|=1;
	     }
           else		//no index mode, gate must be active
             {
	     StoreByte(Ptr0,IRC0CMDWR+2*channel,0x45);     // IOR 0 10 00 1 0 1
             }
           StoreByte(Ptr0,IRC0CMDWR+2*channel,Register);   // IDR

           return HUDAQSUCCESS;
           }

    case HudaqEncMODE:
           switch((int)value)
             {
             case HudaqEncMODEIRC:        ///< Decode IRC connected to inputa A and B (default)
                xEncOptions->Quadrature = 3;
                break;
             case HudaqEncMODERISING:     ///< Count up on A rising edge
                xEncOptions->Quadrature = 0;
                break;

             case HudaqEncMODEFALLING:
             case HudaqEncMODEEITHER:
                return HUDAQNOTSUPPORTED; // these modes are not supported by MF614
	     default:
		return HUDAQBADARG;
             }
           Register = 0x20 | (xEncOptions->Quadrature<<3);  // CMR 0 01 QQ 0 0 0
           StoreByte(Ptr0,IRC0CMDWR+2*channel,Register);    // CMR
           return HUDAQSUCCESS;

    case HudaqEncCOUNTCONTROL:
           if(HudaqEncCOUNTENABLE || HudaqEncCOUNTI1) return HUDAQSUCCESS;
           return HUDAQFAILURE;

    }

return HUDAQNOTSUPPORTED;
}


static HUDAQSTATUS MF614EncReset(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel>=ENC_CHANNELS) return HUDAQBADARG;

  StoreByte(DevRecord->DrvRes.Resources.MemResources[1].Base,
            IRC0CMDWR+2*channel,0x03);    // reset BP & CNTR  0 00 00 01 1
return HUDAQSUCCESS;
}


static int MF614EncRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr0;
int IrcRead;
int ircaddr;


  if (channel>=ENC_CHANNELS) return 0;

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  StoreByte(Ptr0, IRC0CMDWR + 2*channel, 0x11);    // CNTR -> OL, reset BP

  ircaddr = IRC0DATARD + 2*channel;
  IrcRead = GetByte(Ptr0,ircaddr);
  IrcRead += GetByte(Ptr0,ircaddr)<<8;
  IrcRead += GetByte(Ptr0,ircaddr)<<16;
  if (IrcRead>8388607) IrcRead -= 16777216;

  if (((MF614_Private *)DevRecord->DrvRes.DriverData)->EncOptions[channel].ResetOnRead)
    {
    StoreByte(Ptr0,IRC0CMDWR+2*channel,0x03);    // reset BP & CNTR  0 00 00 01 1
    }

return(IrcRead);
}


/****************************************************************
 *                                                              *
 *                COUNTERS PWM + Count + Step                   *
 *                                                              *
 ****************************************************************/

/** This PWM procedure automatically sets prescallers. Change of a prescaller
  causes glitch. But a glitch is never emitted when a duty cycle is changed. */
static HUDAQSTATUS MF614PWMWrite(const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle)
{
unsigned __int32 T1,T2;
double T;
size_t Ptr0;
int Prescaler;
Counter614 *pCtrCache;
HUDAQSTATUS RetCode = HUDAQSUCCESS;


  if (channel>=CTR_CHANNELS) return HUDAQBADARG;
  pCtrCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  if (dutycycle>=1)
    {
    if (pCtrCache->Mode != OUT_1)
      {
      if (pCtrCache->Mode != PWM)
        {       /* Counter Mode Register must be initialized before switching output to 1. */
        *(int *)&pCtrCache->CTC = 0xB62;               // 000 0 1011 | 0 1 1 0 0 010
        pCtrCache->CTC.CountSource = 0x0F;             // maximal acceptable prescaller
        StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
        StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
        StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);
        }
      pCtrCache->Mode = OUT_1;
      StoreByte(Ptr0, TCP, 0xC0|(1<<channel));   // disarm
      StoreByte(Ptr0, TCP, 0xE8|(channel+1));    // set toggle out
      }
    return HUDAQSUCCESS;
    }
  if (dutycycle<=0)
    {
    if (pCtrCache->Mode != OUT_0)
      {
      pCtrCache->Mode = OUT_0;
      StoreByte(Ptr0, TCP, 0xC0|(1<<channel));  // disarm
      StoreByte(Ptr0, TCP, 0xE0|(channel+1));   // clear toggle out
      }
    return HUDAQSUCCESS;
    }

  T = MASTERFREQUENCY / frequency;
  if(T<1) RetCode = HUDAQPARTIAL;

  Prescaler=0xB;
  while(T>65535)    // T1+T2
    {
    T /= 10;
    Prescaler++;
    if(Prescaler>0x0F)          //prescaller overflow (0x0F;  0x11 when aux mode is introduced)
      {                         // 0x0F means 3mHz -> 5.5min
      T=65535;                  // maximal acceptable T; minimal frequency
      Prescaler=0x0F;           // maximal acceptable prescaller
      RetCode = HUDAQPARTIAL;
      break;
      }
    }

  T1 = max(1, (long)floor(T*dutycycle + 0.5));
  T2 = max(1, (long)(T-T1) );

     /* A counter must be disarmed before reloading when prescaler is changed. */
  if(Prescaler != pCtrCache->CTC.CountSource)
    {
    if (pCtrCache->Mode!=OUT_0 && pCtrCache->Mode!=OUT_1)
      StoreByte(Ptr0, TCP, 0xC0|(1<<channel));   // disarm (OUT_0 and OUT_1 modes are already disarmed)
    }

  if(pCtrCache->ACache!=T2)     // Load register
    {
    StoreByte(Ptr0, TCP, (0x09+channel));
    StoreByte(Ptr0, TDP, T2);
    StoreByte(Ptr0, TDP, ((T2)>>8));
    pCtrCache->ACache=T2;
    }

  if(pCtrCache->BCache!=T1)     // Hold register
    {
    StoreByte(Ptr0, TCP, (0x11+channel));
    StoreByte(Ptr0, TDP, T1);
    StoreByte(Ptr0, TDP, ((T1)>>8));
    pCtrCache->BCache=T1;
    }

  if (pCtrCache->Mode != PWM || Prescaler != pCtrCache->CTC.CountSource)
    {
    *(int *)&pCtrCache->CTC = 0xB62;  // 000 0 1011 | 0 1 1 0 0 010
    pCtrCache->CTC.CountSource = Prescaler;
    if(Prescaler>0xF) pCtrCache->CTC.CountSourceEx = 1;

    StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
    StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
    StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

          //I am too lazy to set aux mode register
    //StoreByte(Ptr0, TCP, AUX_TABLE[channel])
    //StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>16);
    //StoreByte(Ptr0, TDP, 0);

    StoreByte(Ptr0, TCP, 0xE0|(channel+1));        // clear toggle out
    StoreByte(Ptr0, TCP, 0x60 | (1<<channel));     // load & arm the selected counter

    pCtrCache->Mode = PWM;
    }

  return RetCode;
}


static HUDAQSTATUS MF614CtrReset(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr0;
Counter614 *pCtrCache;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;
  pCtrCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  if (pCtrCache->Mode != Counting)
    {
    StoreByte(Ptr0, TCP, 0xC0| (1<<channel));      // disarm

    if(pCtrCache->ACache!=0)                       // store 0 into A register
      {
      StoreByte(Ptr0, TCP, (0x9+channel));         // Load Register
      StoreByte(Ptr0, TDP, 0);
      StoreByte(Ptr0, TDP, 0);
      pCtrCache->ACache=0;
      }

    *(int *)&pCtrCache->CTC = 0x12A;               // 000 0 0001 | 0 0 1 0 1 010
    pCtrCache->CTC.CountSource = channel+1;

    StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
    StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
    StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

    StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
    pCtrCache->Mode = Counting;
    }

  StoreByte(Ptr0, TCP, 0x40 | (1<<channel));     // load the selected counter, A register contains zero

return HUDAQSUCCESS;
}


static double MF614CtrGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
size_t Ptr0;
Counter614 *pCtrCache;

  if (channel>=CTR_CHANNELS) return WRONG_VALUE;
  pCtrCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  switch(param)
    {
    case HudaqCtrRESETONREAD:
           return(pCtrCache->ResetOnRead);

    case HudaqPwmCLOCKSOURCE:
    case HudaqCtrCLOCKSOURCE:
           if (pCtrCache->CTC.CountSource == channel+1)
             {
             if (pCtrCache->CTC.EdgeMode==1)
                return HudaqCtrCLOCKINFALLING;
             else
                return HudaqCtrCLOCKINRISING;
             }
           switch(pCtrCache->CTC.CountSource)
             {
             case 0:  if (pCtrCache->CTC.EdgeMode==1)
                        return HudaqCtrCLOCKPREVFALLING;
                      else
                        return HudaqCtrCLOCKPREVRISING;

             case 11: return HudaqCtrCLOCK20MHz;   //1011-F1
             case 12: return HudaqCtrCLOCK2MHz;    //1100-F2
             case 13: return HudaqCtrCLOCK200kHz;  //1101-F3
             case 14: return HudaqCtrCLOCK20kHz;   //1110-F4
             case 15: return HudaqCtrCLOCK2kHz;    //1111-F5
             }
         return -1;  /*Unknown mode*/

    case HudaqPwmGATESOURCE:
    case HudaqCtrGATESOURCE:
          if (pCtrCache->CTC.GateMode==0) return HudaqCtrGATEHIGH;
          return HudaqCtrGATEINPUT;

    case HudaqPwmGATEPOLARITY:
    case HudaqCtrGATEPOLARITY:
          switch (pCtrCache->CTC.GateControl)
            {
            case 0:
            case 5: return 0;	// 0 - low level gate disables counting;
            case 4: return 1;   // 1 - high level gate disables counting.
	    default: return 0;	// ??? Hudaqlib does not support these modes.
	    }

    case HudaqCtrDIRECTION:
          return pCtrCache->CTC.Direction;

    case HudaqCtrLOADTOGGLE:
          return pCtrCache->CTC.ReloadMode;

    case HudaqCtrREPETITION:
          return pCtrCache->CTC.RepeatMode;

    case HudaqPwmOUTPUTCONTROL:
    case HudaqCtrOUTPUTCONTROL:
	 return HudaqCtrOUTPUTNORMAL;

    case HudaqCtrTRIGSOURCE:
         return HudaqCtrTRIGDISABLE;

    case HudaqPwmFILTER:
    case HudaqCtrFILTER:
         return(0);

    case HudaqPwmNUMCHANNELS:
    case HudaqCtrNUMCHANNELS:
         return CTR_CHANNELS;
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS MF614CtrSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
size_t Ptr0;
Counter614 *pCtrCache;
Counter614Control ShadCTC;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;
  pCtrCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  if (pCtrCache->Mode != Counting)
    MF614CtrReset(DevRecord,channel);     // Enforce switching into a counter mode

  switch(param)
    {
    case HudaqCtrRESETONREAD:
           pCtrCache->ResetOnRead = (int)value;
           return HUDAQSUCCESS;

    case HudaqPwmCLOCKSOURCE:
    case HudaqCtrCLOCKSOURCE:
           ShadCTC = pCtrCache->CTC;
           switch((int)value)
             {
             case HudaqCtrCLOCKINRISING:
                ShadCTC.EdgeMode = 0;
                ShadCTC.CountSource = channel+1;
                break;
             case HudaqCtrCLOCKINFALLING:
                ShadCTC.EdgeMode = 1;
                ShadCTC.CountSource = channel+1;
                break;
             case HudaqCtrCLOCKPREVRISING:
                ShadCTC.EdgeMode = 0;
                ShadCTC.CountSource = 0;
                break;
             case HudaqCtrCLOCKPREVFALLING:
                ShadCTC.EdgeMode = 1;
                ShadCTC.CountSource = 0;
                break;
             case HudaqCtrCLOCK20MHz:    //1011-F1
                ShadCTC.CountSource=11;
                break;
             case HudaqCtrCLOCK2MHz:     //1100-F2
                ShadCTC.CountSource=12;
                break;
             case HudaqCtrCLOCK200kHz:   //1101-F3
                ShadCTC.CountSource=13;
                break;
             case HudaqCtrCLOCK20kHz:    //1110-F4
                ShadCTC.CountSource=14;
                break;
             case HudaqCtrCLOCK2kHz:     //1111-F5
                ShadCTC.CountSource=15;
                break;

             default:
                return HUDAQFAILURE;
             }

           if(*(int *)&pCtrCache->CTC != *(int *)&ShadCTC)
             {
             StoreByte(Ptr0, TCP, 0xC0| (1<<channel));      // disarm

             *(int *)&pCtrCache->CTC = *(int *)&ShadCTC;
             StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
             StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
             StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

             StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
             }
           return HUDAQSUCCESS;

    case HudaqCtrOUTPUTCONTROL:
           if( (int)value == HudaqCtrOUTPUTNORMAL) return HUDAQSUCCESS;
           return HUDAQBADARG;

    case HudaqPwmGATEPOLARITY:
    case HudaqCtrGATEPOLARITY:
           switch ((int)value)
            {
            case 0: if (pCtrCache->CTC.GateMode==0 || pCtrCache->CTC.GateMode==5) return HUDAQSUCCESS;
		    pCtrCache->CTC.GateMode=5;
                    break;
            case 1: if (pCtrCache->CTC.GateMode==4) return HUDAQSUCCESS;
		    pCtrCache->CTC.GateMode=4;
                    break;
	    default: return HUDAQBADARG;
	    }
          StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
          StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
          StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

          StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
          return HUDAQSUCCESS;

    case HudaqPwmGATESOURCE:
    case HudaqCtrGATESOURCE:
          switch ((int)value)
            {
            case HudaqCtrGATEHIGH:
               if (pCtrCache->CTC.GateMode==0) return HUDAQSUCCESS;
               pCtrCache->CTC.GateMode=0;
               break;
            case HudaqCtrGATEINPUT:
               if (pCtrCache->CTC.GateControl==0)
                 pCtrCache->CTC.GateControl=5;	    //activate low level gate N
               else
		 {
		 if (pCtrCache->CTC.GateMode==1) return HUDAQSUCCESS;  //gate is now active
		 }
               pCtrCache->CTC.GateMode=1;
               break;
            default:
               return HUDAQBADARG;
            }

          StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
          StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
          StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

          StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
          return HUDAQSUCCESS;

    case HudaqCtrDIRECTION:
          if(value>=2) return HUDAQBADARG;
          if (pCtrCache->CTC.Direction!=(int)value);
	    {
            pCtrCache->CTC.Direction = (int)value;
            StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

            StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
            }
          return HUDAQSUCCESS;

    case HudaqCtrLOADTOGGLE:
	  if(value>=2) return HUDAQBADARG;
          if (pCtrCache->CTC.ReloadMode!=(int)value);
	    {
            pCtrCache->CTC.ReloadMode=(int)value;
            StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

            StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
            }
          return HUDAQSUCCESS;

    case HudaqCtrREPETITION:
          if(value>=2) return HUDAQBADARG;
          if (pCtrCache->CTC.RepeatMode!=(int)value);
	    {
            pCtrCache->CTC.RepeatMode = (int)value;
            StoreByte(Ptr0, TCP, channel+1);               // select counter mode register
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC);
            StoreByte(Ptr0, TDP, *(int *)&pCtrCache->CTC>>8);

            StoreByte(Ptr0, TCP, 0x20 |(1<<channel));      // arm
            }
          return HUDAQSUCCESS;

     case HudaqCtrTRIGSOURCE:
          if ((int)value == HudaqCtrTRIGDISABLE) return HUDAQSUCCESS;
          return HUDAQBADARG;

     case HudaqPwmFILTER:
     case HudaqCtrFILTER:
          if ((int)value == 0) return HUDAQSUCCESS;
          return HUDAQBADARG;
    }

  return HUDAQNOTSUPPORTED;
}


static int MF614CtrRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr0;
Counter614 *pCtrCache;

  if (channel>=CTR_CHANNELS) return 0;
  pCtrCache = &((MF614_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  if (pCtrCache->Mode != Counting)
    {
    MF614CtrReset(DevRecord,channel);
    return 0;
    }

  StoreByte(Ptr0, TCP, 0xA0|(1<<channel));         // save counter
  StoreByte(Ptr0, TCP, 0x11+channel);              // read value hold
  pCtrCache->BCache = GetPackedWord(Ptr0,TDP);     // Hold register is influenced

  if(pCtrCache->ResetOnRead)
    {
    StoreByte(Ptr0, TCP, 0x40 | (1<<channel));     // load the selected counter, A register contains zero
    }

return(pCtrCache->BCache);
}


/** Handle stepping motor through MF614. */
static HUDAQSTATUS MF614StepWrite(const DeviceRecord *DevRecord, unsigned channel, int position)
{
size_t Ptr0;
int maskC, maskF, status, count;
__int32 N;
int NewSteps;
int digin;
int State=0;
MF614_Private *Cache;
Stepper614 *CacheStep;
DWORD NewTime;
double NewFrequency;

  NewTime=timeGetTime();

  if (channel>=STEP_CHANNELS) return HUDAQBADARG;
  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;
  CacheStep = &(Cache->step[channel]);

  maskF=1<<(2*channel);
  maskC=maskF<<1;

  if (Cache->counter[2*channel].Mode != StepperPWM)
    {           //check for the first time initialization of slave counter
    Cache->counter[2*channel].Mode = StepperPWM;

    StoreByte(Ptr0, TCP, 2*channel+1);                   // select counter 1 mode register
    StoreByte(Ptr0, TDP, 0x22);
    StoreByte(Ptr0, TDP, 0x8C);     // mode E, F2 source (2MHz), toggle output

    N = (int) (MASTERFREQUENCY/(2*10*CacheStep->Fmin));
    if(Cache->counter[2*channel].ACache!=N)
      {
      StoreByte(Ptr0, TCP, (0x9+2*channel));    //Load register
      StoreByte(Ptr0, TDP, N);
      StoreByte(Ptr0, TDP, N>>8);
      Cache->counter[2*channel].ACache = N;
      }

    StoreByte(Ptr0, TCP, 0xE1+2*channel);                // reset output

    State|=1;
    }

  if (Cache->counter[2*channel+1].Mode != StepperMaster)
    {      //check for the first time initialization of master counter
    Cache->counter[2*channel+1].Mode = StepperMaster;

    StoreByte(Ptr0, TCP, 2*channel+2);                   // select counter 2 mode register
    StoreByte(Ptr0, TDP, 0x02);
    StoreByte(Ptr0, TDP, 0x12+2*channel);                // mode A, output TC toggle
                // Internal TC(n-1) do not work, I suspect internal hazards.

    if(Cache->counter[2*channel+1].ACache!=1)
      {
      StoreByte(Ptr0, TCP, (0x9+2*channel+1));           //Load register
      StoreByte(Ptr0, TDP, 1);
      StoreByte(Ptr0, TDP, 0);
      Cache->counter[2*channel+1].ACache = 1;
      }

    StoreByte(Ptr0, TCP, 0xE1 + 2*channel+1);            // reset output

    State|=2;
    }

  if (State!=0)         //check for the first time initialization
    {
    CacheStep->Direction=0;
    CacheStep->LastPosition=0;
    CacheStep->ChainedPosition=0;
    CacheStep->Frequency=0;

    StoreByte(Ptr0, TCP, 0x4F);                      // load counters
    StoreByte(Ptr0, TCP, 0x25);                      // arm counters
    }

              //********* check for end switches **********
  digin = GetByte(DevRecord->DrvRes.Resources.MemResources[1].Base,DIN);
  if( ((digin & (1<<(2*channel)))==0 && (CacheStep->Direction>0)) ||
    ( ((digin & (1<<(2*channel+1)))==0) && (CacheStep->Direction<0)) )  // end switch
   {
    StoreByte(Ptr0, TCP, 0xA0|maskF|maskC);         // save both counters
    StoreByte(Ptr0, TCP, 0x11+2*channel+1);         // read count hold
    NewSteps = GetPackedWord(Ptr0,TDP);
    Cache->counter[2*channel+1].BCache = NewSteps++;

    StoreByte(Ptr0, TCP, 0x1F);                     // read status register
    status=GetPackedWord(Ptr0,TDP);
    if((status & (0x04<<2*channel)) == 0)           // stopped?
      NewSteps = 0;

    StoreByte(Ptr0, TCP, 0xE2+2*channel);           // reset output
    StoreByte(Ptr0, TCP, 0xC0|maskC);               // disarm

    CacheStep->LastPosition -= NewSteps * Cache->step[channel].Direction;   //actualize real position

    CacheStep->Frequency = 0;
    CacheStep->Direction = 0;
    CacheStep->TimeStamp = NewTime;
    return HUDAQSUCCESS;
   }

        //***** actualize step count ******
  if (CacheStep->LastPosition!=position)
    {                                     //amount of steps to do
    if( (State&3)!=0 ) //the first time initialization
      {
      NewSteps = 0;
      }
    else               // counter already runs
      {
      StoreByte(Ptr0, TCP, 0x1F);
      status=GetPackedWord(Ptr0,TDP);
      if((status & (0x04<<2*channel)) ==0)               // stopped
        NewSteps = 0;
      else
        {
        StoreByte(Ptr0, TCP, 0xA0|maskF|maskC);          // save both counters
        StoreByte(Ptr0, TCP, 0x11+2*channel);            // read frequency hold
        Cache->counter[2*channel].BCache =
          status = count = GetPackedWord(Ptr0,TDP);
        while(count < 40)           // wait until safe to change pulse count
          {
          StoreByte(Ptr0, TCP, 0xA0|maskF|maskC);        // save both counters
          StoreByte(Ptr0, TCP, 0x11+2*channel);          // read frequency hold
          Cache->counter[2*channel].BCache =
            count = GetPackedWord(Ptr0,TDP);
          if(status==count)
            {
            Cache->counter[2*channel+1].BCache = 0;
            NewSteps = 0;
            goto UnexpectedStop;                         //unexpected stop of frequency counter
            }
          status=count;
          }
        StoreByte(Ptr0, TCP, 0x11+2*channel+1);          // read count hold
        NewSteps = GetPackedWord(Ptr0,TDP);
        Cache->counter[2*channel+1].BCache = NewSteps++;
UnexpectedStop: ;
        }
      }

    if(NewSteps==0)
      {
      CacheStep->Direction=0;   //motor is already stopped.
      CacheStep->Frequency=0;
      }

          // CurrentPosition = LastPosition - |x|*Direction
          // CurrentPosition = NewPosition - NewSteps
          //   ===> NewSteps = NewPosition - LastPosition + |x|*Direction
    NewSteps = position - CacheStep->LastPosition + NewSteps*CacheStep->Direction;


    if (NewSteps>2)     //It is impossible to realise less than 3 steps???
      {
      if (CacheStep->Direction==-1)
        {       //direction has been reversed, calculate a false target
          //NewSteps = Fmin(channel)*Fmin(channel)+Frequency[channel]*Frequency[channel])/(2*Acc(channel)) - 1;
                // \todo   Study behaviour of this feature. It lets stepper to finish in oposite direction.
        goto TargetIsFixed;
        }
      CacheStep->LastPosition = position;   //actualize position cache
      CacheStep->Direction=1;
      if( NewSteps > 0xFFFF )
        {
        CacheStep->LastPosition -= NewSteps-0xFFFF;
        NewSteps = 0xFFFF;
        }
      StoreByte(Ptr0, TCP, 0xC0|maskC);                     // disarm count
      StoreByte(Ptr0, TCP, 0x0A+2*channel);                 // count load register
      StoreByte(Ptr0, TDP, NewSteps-1);
      StoreByte(Ptr0, TDP, (NewSteps-1)>>8);
      State |= 4;                           //reload & run counter
      }

    if (NewSteps<-2)   //It is impossible to realise less than 3 steps???
      {
      if (CacheStep->Direction==1)         //if (Frequency[channel]>Fmin[channel])
        {       //direction has been reversed, calculate a false target
          //NewSteps = Fmin(channel)*Fmin(channel)+Frequency[channel]*Frequency[channel])/(2*Acc(channel)) - 1;
                        // \todo   Study behaviour of this feature. It lets stepper to finish in oposite direction.
        goto TargetIsFixed;
        }
      CacheStep->LastPosition = position;        //actualize position cache
      CacheStep->Direction = -1;
      NewSteps = -NewSteps;
      if( NewSteps > 0xFFFF )
        {
        CacheStep->LastPosition += NewSteps-0xFFFF;
        NewSteps = 0xFFFF;
        }
      StoreByte(Ptr0, TCP, 0xC0|maskC);                       // disarm count
      StoreByte(Ptr0, TCP, 0x0A+2*channel);                   // count load register
      StoreByte(Ptr0, TDP, NewSteps-1);
      StoreByte(Ptr0, TDP, (NewSteps-1)>>8);
      State |= 4;                                //reload & run counter
      }

    if (NewSteps==0)
      {
      //CacheStep->LastPosition;   does not need to be actualised
      CacheStep->Direction=0;
      }
    }

TargetIsFixed:
  CacheStep->ChainedPosition = position;   // chained position makes sense only if direction is reversed


     //************ set frequency - (lastreq contains amount of remainning steps) ***********
  if (CacheStep->Acc<=0)
    CacheStep->Frequency=CacheStep->Fmax;        // full acceleration when acceleration is negative
  else
    {
    if (CacheStep->Frequency==0)
      {
      CacheStep->Frequency = CacheStep->Fmin;
      }
    else
      {
      if ((CacheStep->Acc > 0) && (NewSteps > (CacheStep->Fmin*CacheStep->Fmin+CacheStep->Frequency*CacheStep->Frequency)/(2*CacheStep->Acc)))
        NewFrequency = CacheStep->Frequency + CacheStep->Acc*abs((long)NewTime-(long)CacheStep->TimeStamp)/1000.0;
      else
        NewFrequency = CacheStep->Frequency + CacheStep->Acc*abs((long)NewTime-(long)CacheStep->TimeStamp)/1000.0;

      if (NewFrequency > CacheStep->Fmax) NewFrequency = CacheStep->Fmax;
      if (NewFrequency < CacheStep->Fmin) NewFrequency = CacheStep->Fmin;

      if (NewFrequency>(CacheStep->Frequency+MAXIMAL_STEP_INCREMENT)) //maximal increment threshold
        {
        CacheStep->Frequency += MAXIMAL_STEP_INCREMENT;
        }
      else
        CacheStep->Frequency = NewFrequency;
      }
    }

  N = (__int32)(MASTERFREQUENCY / (10*CacheStep->Frequency));   //set a real frequency
  N = max(1,N/2);
  if(N>65535) N=65535;

  if(Cache->counter[2*channel].ACache != N)
   {
   StoreByte(Ptr0, TCP, 0x09+2*channel);
   StoreByte(Ptr0, TDP, N);
   StoreByte(Ptr0, TDP, N>>8);
   Cache->counter[2*channel].ACache = N;
   }

    //write stepper direction to digital output
  if(CacheStep->Direction!=0)  // only when stepper is expected to be running
    MF614DOWriteBit(DevRecord,0,channel,CacheStep->Direction>0);

    //start counters
  if ( (State&4) != 0 )
    {
    StoreByte(Ptr0, TCP, 0x60|maskC);                // load and arm count
    StoreByte(Ptr0, TCP, 0xEA+2*channel);            // set toggle out -> start
    }

  CacheStep->TimeStamp = NewTime;                    //refresh timestamp
  return HUDAQSUCCESS;
}



static HUDAQSTATUS MF614StepSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF614_Private *Cache;
size_t Ptr0;
int status, RemainingSteps;

  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=STEP_CHANNELS) return HUDAQBADARG;

  if (Cache->counter[2*channel].Mode != StepperPWM || Cache->counter[2*channel+1].Mode != StepperMaster)
    MF614StepWrite(DevRecord,channel,0);     //call stepper initialization if it is not called before

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  switch(param)
    {
    case HudaqStepFMIN:
        Cache->step[channel].Fmin = value;
        return HUDAQSUCCESS;
    case HudaqStepFMAX:
        Cache->step[channel].Fmax = value;
        return HUDAQSUCCESS;
    case HudaqStepACCELERATION:
        Cache->step[channel].Acc = value;
        return HUDAQSUCCESS;

    case HudaqStepCURRENTPOSITION:
        StoreByte(Ptr0, TCP, 0xA0|(2<<(2*channel)));    // save counter
        StoreByte(Ptr0, TCP, 0x11+2*channel+1);         // read count hold
        RemainingSteps = GetPackedWord(Ptr0,TDP);
        Cache->counter[2*channel+1].BCache = RemainingSteps++;

        StoreByte(Ptr0, TCP, 0x1F);                     // read status register
        status = GetPackedWord(Ptr0,TDP);
        if((status & (0x04<<2*channel)) == 0)           // stopped?
          RemainingSteps = 0;

        // CurrentPosition = LastPosition - Direction * AmountOfSteps
        //     NewPosition = NewLastPosition - Direction * AmountOfSteps
        // ==>  NewLastPosition = NewPosition + Direction * AmountOfSteps
        Cache->step[channel].LastPosition =
           (int)value + Cache->step[channel].Direction * RemainingSteps;
        return HUDAQSUCCESS;

    }

return HUDAQNOTSUPPORTED;
}


static double MF614StepGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF614_Private *Cache;
size_t Ptr0;
int status, RetVal;

  Cache = (MF614_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=2) return WRONG_VALUE;

  if (Cache->counter[2*channel].Mode != StepperPWM || Cache->counter[2*channel+1].Mode != StepperMaster)
    MF614StepWrite(DevRecord,channel,0);     //call stepper initialization if it is not called before

  Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;

  switch(param)
    {
    case HudaqStepFMIN:
        return(Cache->step[channel].Fmin);
    case HudaqStepFMAX:
        return(Cache->step[channel].Fmax);
    case HudaqStepACCELERATION:
        return(Cache->step[channel].Acc);
    case HudaqStepCURRENTPOSITION:      // CurrentPosition = LastPosition - Direction * AmountOfSteps
        StoreByte(Ptr0, TCP, 0xA0|(2<<(2*channel)));    // save counter
        StoreByte(Ptr0, TCP, 0x11+2*channel+1);         // read count hold
        RetVal = GetPackedWord(Ptr0,TDP);
        Cache->counter[2*channel+1].BCache = RetVal++;

        StoreByte(Ptr0, TCP, 0x1F);                     // read status register
        status=GetPackedWord(Ptr0,TDP);
        if((status & (0x04<<2*channel)) == 0)           // stopped?
          return(0);

        return(Cache->step[channel].LastPosition -
               Cache->step[channel].Direction * RetVal);

    case HudaqStepREMAININGSTEPS:
        StoreByte(Ptr0, TCP, 0xA0|(2<<(2*channel)));    // save counter
        StoreByte(Ptr0, TCP, 0x11+2*channel+1);         // read count hold
        RetVal = GetPackedWord(Ptr0,TDP);
        Cache->counter[2*channel+1].BCache = RetVal++;

        StoreByte(Ptr0, TCP, 0x1F);                     // read status register
        status=GetPackedWord(Ptr0,TDP);
        if((status & (0x04<<2*channel)) == 0)           // stopped?
          return(0);
        return(RetVal);

    case HudaqStepTARGETPOSITION:
        return(Cache->step[channel].ChainedPosition);
    case HudaqStepNUMCHANNELS:
        return STEP_CHANNELS;
    }

return WRONG_VALUE;
}


/****************************************************************
 *                                                              *
 *                    GET/SET PARAMETERS                        *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS MF614SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  //case HudaqDI:
  //case HudaqDO:
  case HudaqAI:  return MF614AISetParameter(DevRecord,channel,param,value);
  case HudaqAO:  return MF614AOSetParameter(DevRecord,channel,param,value);
  case HudaqEnc: return MF614EncSetParameter(DevRecord,channel,param,value);
  case HudaqCtr: return MF614CtrSetParameter(DevRecord,channel,param,value);
  case HudaqStep:return MF614StepSetParameter(DevRecord,channel,param,value);
  }
return HUDAQNOTSUPPORTED;
}


static HUDAQSTATUS AD612SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  //case HudaqDI:
  //case HudaqDO:
  case HudaqAI:  return MF614AISetParameter(DevRecord,channel,param,value);
  case HudaqAO:  return MF614AOSetParameter(DevRecord,channel,param,value);
  //case HudaqEnc:
  //case HudaqCtr:
  //case HudaqStep:
  }
return HUDAQNOTSUPPORTED;
}


static double MF614GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return MF614DIOGetParameter(channel,param);
  case HudaqAI:  return MF614AIGetParameter(DevRecord,channel,param);
  case HudaqAO:  return MF614AOGetParameter(DevRecord,channel,param);
  case HudaqEnc: return MF614EncGetParameter(DevRecord,channel,param);
  case HudaqCtr: return MF614CtrGetParameter(DevRecord,channel,param);
  case HudaqStep:return MF614StepGetParameter(DevRecord,channel,param);
  }
return WRONG_VALUE;
}


static double AD612GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return MF614DIOGetParameter(channel,param);
  case HudaqAI:  return MF614AIGetParameter(DevRecord,channel,param);
  case HudaqAO:  return MF614AOGetParameter(DevRecord,channel,param);
  //case HudaqEnc:
  //case HudaqCtr:
  //case HudaqStep:
  }
return WRONG_VALUE;
}


static const HudaqRange *MF614QueryRange(const DeviceRecord *DevRecord, HudaqSubsystem S, unsigned item)
{
  switch(S & HudaqSubsystemMASK)
    {
    case HudaqAI:  if(item>=4) return NULL;
                   return &MF614RangeAIO[item];
    case HudaqAO:  if(item>=1) return NULL;
                   return &MF614RangeAIO[0];
    }
  return NULL;
}




/****************************************************************
 *                                                              *
 *                       INITIALIZATION                         *
 *                                                              *
 ****************************************************************/

/** Initialize subset of hardware that is supported by both MF614 & AD612. */
static void InitAiAoDiDo(size_t Ptr0, size_t Ptr1, MF614_Private *Cache)
{
int i;

      /* Initialization of AD's */
    for(i=0;i<AD_CHANNELS;i++)
      {
      Cache->AIOptions[i].Range=1;    // 10V
      Cache->AIOptions[i].Bipolar=1;  // + -
      Cache->AIOptions[i].Raw=0;
      }

      /* Initialization of DA's */
    for(i=0;i<DA_CHANNELS;i++)
      {
      Cache->DAOptions[i].Raw = 0;
      Cache->DAOptions[i].LoCache = 0;
      Cache->DAOptions[i].HiCache = 0x8;

      StoreByte(Ptr0, DA0LO + 2*i, 0);
      StoreByte(Ptr0, DA0HI + 2*i, 0x8);
      }
    i = GetByte(Ptr0, DALATCHEN);            // update the output

      /* Initialization of digital outputs */
    Cache->DoutCache = 0;
    StoreByte(Ptr0, DOUT, 0);
}


/** Internal initialization function that should be called from HudaqOpenDevice
   * \todo Exclusive access is only partially implemented. */
static int Init614(DeviceRecord *DevRecord, int IniOptions)
{
size_t Ptr0, Ptr1, Ptr2;
MF614_Private *Cache;
int i;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<2) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(MF614_Private)) return -4;

  DevRecord->pCT = &CT614;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {                           //initialize hardware only if no other application is running
    Ptr0 = DevRecord->DrvRes.Resources.MemResources[1].Base;
    Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base + 0x400;
    Ptr2 = DevRecord->DrvRes.Resources.MemResources[0].Base;

      /* Initialization of AD's, DA's, DI and DO. */
    InitAiAoDiDo(Ptr0, Ptr1, Cache);

      /* initialization IRC */
    for (i=0; i<ENC_CHANNELS; i++)
      {
      StoreByte(Ptr0,IRC0CMDWR+2*i,0x03);    // reset BP & CNTR  0 00 00 01 1
      StoreByte(Ptr0, IRC0DATAWR+2*i, 9);    // PR0=9; program filter to 250 kHz (20MHz/9)
      StoreByte(Ptr0, IRC0CMDWR+2*i, 0x18);  // PR0 -> PSC
      Cache->EncOptions[i].Filter = 9;

      StoreByte(Ptr0,IRC0CMDWR+2*i,0x38);    // CMR 0 01 11 0 0 0
      Cache->EncOptions[i].Quadrature = 3;   // ----------^

      StoreByte(Ptr0,IRC0CMDWR+2*i,0x45);    // IOR 0 10 00 1 0 1 (use gate)
      Cache->EncOptions[i].Index = 0;

      StoreByte(Ptr0,IRC0CMDWR+2*i,0x60);    // IDR 0 11 00 0 0 0 (index pulse polarity)
      Cache->EncOptions[i].Polarity = 0;     // --------------^ ^
      Cache->EncOptions[i].Level = 1;        // ----------------|

      Cache->EncOptions[i].ResetOnRead = 0;
      }

      /* Initialization of counters */
        /* set counters mode */
    StoreByte(Ptr0,TCP,0xFF);                 // master reset
    StoreByte(Ptr0,TCP,0x17);                 // master mode register
    StoreByte(Ptr0,TDP,0x00);
    StoreByte(Ptr0,TDP,0x80);                 // BCD prescaler
    for(i=0;i<CTR_CHANNELS;i++)
      {
      Cache->counter[i].Mode = Unspecified;
      Cache->counter[i].ACache = 0;
      Cache->counter[i].BCache = 0;
      }

    /* Initialization of stepper motors courtesy */
    for(i=0;i<STEP_CHANNELS;i++)
      {
      Cache->step[i].LastPosition = 0;
      Cache->step[i].ChainedPosition = 0;
      Cache->step[i].Direction = 0;

      Cache->step[i].Fmin = 1000;
      Cache->step[i].Fmax = 5000;
      Cache->step[i].Acc = 1000;
      }

    }

return 1; //success
}


/** Internal cleanup procedure for MF614 & AD612 */
static void Done614(DeviceRecord *DevRecord)
{
  if (DevRecord==NULL) return;

  DevRecord->pCT = &CtDummy;
}


const CallTable CT614 =
    {
    "MF614", 0x186C, 0x0614,
    Init614,
    Done614,

    MF614SetParameter,
    MF614GetParameter,
    MF614QueryRange,

        // INITIALIZE DI callers
    MF614DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    MF614DOWrite,
    MF614DOWriteBit,
    MF614DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    MF614AIRead,
    GenericAIReadMultiple,
        // INITIALIZE AO callers
    MF614AOWrite,
    MF614AOWriteMultiple,
         // INITIALIZE Enc callers
    MF614EncRead,
    MF614EncReset,
        // INITIALIZE Ctr callers
    MF614CtrRead,
    MF614CtrReset,
        // INITIALIZE PWM callers
    MF614PWMWrite,
    NULL,
    NULL,
        // INITIALIZE Step callers
    MF614StepWrite,
    };


/* Initialization procedure for AD612 card. */
static int Init612(DeviceRecord *DevRecord, int IniOptions)
{
MF614_Private *Cache;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<2) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(MF614_Private)) return -4;

  DevRecord->pCT = &CT612;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {                           //initialize hardware only if no other application is running
      /* Initialization of AD's, DA's, DI and DO. */
    InitAiAoDiDo(DevRecord->DrvRes.Resources.MemResources[1].Base,         //Ptr0
	         DevRecord->DrvRes.Resources.MemResources[1].Base + 0x400, //Ptr1
		 Cache);
    }

return 1; //success
}


const CallTable CT612 =
    {
    "AD612", 0x186C, 0x0612,
    Init612,
    Done614,

    AD612SetParameter,
    AD612GetParameter,
    MF614QueryRange,

        // INITIALIZE DI callers
    MF614DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    MF614DOWrite,
    MF614DOWriteBit,
    MF614DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    MF614AIRead,
    GenericAIReadMultiple,
        // INITIALIZE AO callers
    MF614AOWrite,
    MF614AOWriteMultiple,
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
