/****************************************************************/
/**@file MF624.c:
 * Description: API layer for MF624 card.                       *
 * Dependency: Windows 32, Windows 64 or Linux                  *
 *                Copyright 2006-2007 Humusoft s.r.o.           *
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



/** IRC control register */
typedef struct
{
  unsigned int IRC_MODE:2;      ///< 0-IRC; 01-Rising Edge; 10-Falling Edge; 11-either edges
  unsigned int BLOCK_MODE:2;    ///< 00-enable; 01-disable; 10-count if I=0; 11-10-count if I=1
  unsigned int RESET_MODE:3;    ///< 000-enable; 001b-reset; 010-reset I=0; 011-reset I=1
                                ///< 100-reset on rising I; 101-reset on falling I; 110-reset on either edges I; 111-none
  unsigned int FILTER:1;        ///< 0-off; 1-on
} IRC_Control;


/** Counter control register */
typedef struct
{
  unsigned int Direction:1;     ///< 0-Down; 1-Up
  unsigned int Repetition:1;    ///< 0-One shot; 1-repeat
  unsigned int LoadToggle:1;    ///< 0-Load from A; 1-alternate A and B
  unsigned int OutputToggle:1;  ///< 0-terminal count; 1-toggle
  unsigned int OutputControl:2; ///< 00-output; 01-inverted output; 10-0; 11-1
  unsigned int TriggerSource:2; ///< 00-1; 01-IN; 10-Out_n-1; 11-Out_n+1
  unsigned int TriggerType:2;   ///< 00-disabled; 01-rising edge; 10-falling edge; 11-either egde
  unsigned int ReTrigger:1;     ///< 0-disabled; 1-enabled
  unsigned int GateSource:2;    ///< 00-1; 01-IN; 10-Out_n-1; 11-Out_n+1
  unsigned int GatePolarity:1;  ///< 0-'0' disables counting; 1-'1' disables counting
  unsigned int ClockSource:4;   ///< 0-internal 50MHz; 1-10MHz; 2-1MHz; 3-100kHz; 4-NOTHING
                                ///< 5-Rise IN; 6-Fall IN; 7-Either IN; 8-NOTHING
                                ///< 9-Rise N-1; 10-Fall N-1; 11-Either N-1; 12-NOTHING
                                ///< 13-Rise N+1; 14-Fall N+1; 15-Either N+1
  unsigned int Filter:1;        ///< 0-off; 1-on
} CounterControl;


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
} InternalCounterMode;


/** Symbolic constants for handling Counter Control Register. These cnstants are aimed to
   use by this way: GLOB_CTR_BASE0*(GLOB_CTR_STOP|GLOB_CTR_LOAD) | GLOB_CTR_BASE1*(GLOB_CTR_START) */
#define GLOB_CTR_START      1	 ///< Counter start command
#define GLOB_CTR_STOP       2    ///< Counter stop command
#define GLOB_CTR_LOAD       4    ///< Counter load command
#define GLOB_CTR_RESET      8    ///< Counter reset command
#define GLOB_CTR_OUT_SET   16	 ///< Set counter output toggle bit
#define GLOB_CTR_OUT_RESET 32    ///< Reset counter output toggle bit

#define GLOB_CTR_BASE0         1 ///< bit offset of a 1st counter in GLOB_CTR register
#define GLOB_CTR_BASE1      0x40 ///< bit offset of a 2nd counter in GLOB_CTR register
#define GLOB_CTR_BASE2    0x1000 ///< bit offset of a 3rd counter in GLOB_CTR register
#define GLOB_CTR_BASE3   0x40000 ///< bit offset of a 4th counter in GLOB_CTR register
#define GLOB_CTR_BASE4 0x1000000 ///< bit offset of a 5th counter in GLOB_CTR register


#define DA_CHANNELS   8         ///< Amount of Analog Outputs
#define AD_CHANNELS   8         ///< Amount of Analog Inputs
#define ENC_CHANNELS  4         ///< Amount of Encoders
#define CTR_CHANNELS  5         ///< Amount of counters
#define STEP_CHANNELS 2         ///< Amount of steppers
#define PWM_625_SUBCHANNELS  6  ///< Amount of PWM channel comparators
#define PWM_625_CHANNELS 1	///< Amount of PWM channels



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
} Stepper;



/** Commands are mixed into ::MF625Register. */
typedef enum
{
  MF625_START =                1,  ///< Start PWM counter
  MF625_STOP =                 2,  ///< Stop PWM counter
  MF625_RESET =                8,  ///< Reset PWM counter
  MF625_SHEDULE_UPDATE =  0x2000,  ///< Shedule dual buffer update
  MF625_CANCEL_SHEDULE = 0x10000,  ///< Discard any pending shedule update
  MF625_FORCE_UPDATE =   0x20000,  ///< Update registers from dual buffer now, clear shedule flag like ::MF625_CANCEL_SHEDULE
  MF625_LOAD =          0x100000,  ///< Reload counter with a divider value
  MF625_FORCE_DOWN =    0x200000,  ///< Set counter direction down (counter switches automatically to down, when reaching max value).
  MF625_FORCE_UP =      0x400000,  ///< Set counter direction up (counter switches automatically to up, when reaching zero value).
  MF625_COMMANDS = MF625_START | MF625_STOP | MF625_RESET | MF625_LOAD |
		   MF625_SHEDULE_UPDATE | MF625_CANCEL_SHEDULE | MF625_FORCE_UPDATE |
		   MF625_FORCE_DOWN | MF625_FORCE_UP
} MF625Commands;


/** Counter control register for MF625. */
typedef struct
{
  unsigned int Running:1;	///< R  0- counter is stopped; 1-counter is running
  unsigned int Empty0:1;	///< -
  unsigned int GatePolarity:1;	///< RW 0-'0' disables counting; 1-'1' disables counting
  unsigned int InputWire:1;	///< R	a state of input wire
  unsigned int OutputControl:2; ///< RW 00-output; 01-inverted output; 10-0; 11-1
  unsigned int ClockSource:4;   ///< RW 0-internal 50MHz; 1-10MHz; 2-1MHz; 3-100kHz; 4-NOTHING
                                ///<    5-Rise IN; 6-Fall IN; 7-Either IN; 8-NOTHING
                                ///<    9-Rise N-1; 10-Fall N-1; 11-Either N-1; 12-NOTHING
                                ///<    13-Rise N+1; 14-Fall N+1; 15-Either N+1
  unsigned int Filter:1;        ///< RW 0-off; 1-on
  unsigned int GateSource:2;    ///< RW 00-1; 01-IN; 10-Out_n-1; 11-Out_n+1
  unsigned int UpdatePending:1;	///< R  The update command waits to be realised. Update occurs when counter changes direction frou UP to down.
  unsigned int OutputUDCtrl:2;	///< RW register PWM output polarity control; 00-output; 01-inverted output; 10-0; 11-1
  unsigned int PhaseOut:6;	///< R  Readback of comparators.
  unsigned int UpDown:1;	///< R  A direction of main counter.
  unsigned int Transparent:1;   ///< RW This handles dual buffering: 0-off; 1-on.
  unsigned int Inversions:6;    ///< RW Turn on/off inversion of n'th pwm output subchannel.
  unsigned int Emergency:2;     ///< RW Selection of emergency shutdown, see ::HudaqPwmEmergency.
} MF625Register;


/** Modes of 3 phase PWM counter. */
typedef enum
{
  Unspecified_3f = 0,
  OUT_0_ALL,                    ///< counter is generating 0 permanently on all wires
  OUT_1_ALL,                    ///< counter is generating 1 permanently on all wires
  Phase3PWM,			///< 3 phases + their inversions
  Phase3PWM_prep,		///<   intermediate internal state between something and Phase3PWM
  Phase6PWM,			///< 6 phases
  Phase6PWM_prep,		///<   intermediate internal state between something and Phase6PWM
} MF625Mode;


/** Record related to multiphase PWM. */
typedef struct
{
  __int32 Comparators[PWM_625_SUBCHANNELS]; ///< Cached values for comparators
  __int32 Divider;		///< Cached value of a divider.
  MF625Register CTR;		///< Counter control register cache - see ::MF625Register.
  double DeadBand;		///< Delay between inverted and not inverted phases. Used when InvertedPhase=1.
  double CtrFixPWM;             ///< Fix PWM when switching from low to high frequency, 0 turns off
  MF625Mode Mode;	        ///< 0 - after initialization; 1 phases + inversion; 2 - 6 phases
} PWMMultiple;


/** One record related to counter. */
typedef struct
{
  CounterControl CCache;        ///< Counter state register cache
  __int32 ACache;               ///< Counter A register cache
  __int32 BCache;               ///< Counter B register cache
  InternalCounterMode Mode;	///< Flag of subsystem that uses counter
  double CtrFixPWM;             ///< Fix PWM when switching from low to high frequency, 0 turns off
  int CtrResetCounter:1;        ///< Reset counter after every read
} Counter;


/** One record related to Digital to Analog output channel. */
typedef struct
{
  __int16 Cache;                ///< Cached raw value for digital output
  int Options;                  ///< DOut bit options; bit 0=0 volts, 1-raw
} DaSetup;


#define IrcResetCounter 2       ///< Binary flag for IrcOptions

/** Ranges for both Analog Inputs and Analog Outputs - only one item for MF624. */
static const HudaqRange MF624RangeAIO = {-10,+10};


/** Cache of MF624 state, this cache is shared across all applications inside shared memory. */
typedef struct
{
  UserDataHeader Hdr;                   ///< General device setup

                /* Digital inputs/outputs */
  __int16 DoutCache;                    ///< digital outputs to be cached

                /* Analog inputs/outputs */

  unsigned AIOptions[AD_CHANNELS];	///< Options for analog inputs
  __int16 ADCtrlCache;                  ///< Selected channels for conversion

  DaSetup DA[DA_CHANNELS]; 		///< DA values to be cached

                /* Counters and encoders */

  __int32 EncCache;                     ///< Encoder (IRC counter) state register
  int EncOptions[ENC_CHANNELS];		///< Bit options for encoders

  Counter counter[CTR_CHANNELS];	///< Array that holds state of all counters

  union {
        Stepper step[STEP_CHANNELS];	///< Stepper motor table
        PWMMultiple pwm;		///< MF625 specialized PWM counter
        };
} MF624_Private;

#define MAXIMAL_STEP_INCREMENT 500	///< Maximal speed increase between two calls of StepWrite.


/** Convert values from A/D convertor to a voltage. */
static __inline double MF624_AD2VOLT(signed __int16 x)
{
  return 10.0*((signed __int16)(x<<2))/(double)0x8000;
}


/** Convert voltage to a raw values for D/A convertor. */
static __inline __int16 MF624_VOLT2DA(double y)
{
  y = 0x4000*(y+10)/20;
  if (y>=0x3FFF) return(0x3FFF);
  if (y<=0) return(0x0000);
  return( (__int16)(y) );
}


#define MASTERFREQUENCY 50000000        ///< Internal oscillator frequency is 50MHz



/** Symbolic aliases for available registers inside BADR0. */
typedef enum
{
  INTCSR  = 0x4C,
  GPIOC   = 0x54
} BADR0;

#define GPIOC_BASE  0x024006C0
#define GPIOC_EOLC  0x00020000
#define GPIOC_LDAC  0x00800000
#define GPIOC_DACEN 0x04000000

/** Symbolic aliases for available registers inside BADR1 read. */
typedef enum
{
  ADDATA  = 0x00,
  DIN     = 0x10,
  ADSTART = 0x20,
} BADR1IN;

/** Symbolic aliases for available registers inside BADR1 write. */
typedef enum
{
  ADCTRL = 0x0,
  DOUT   = 0x10,
  DADATA = 0x20,
} BADR1OUT;

/** Symbolic aliases for available registers inside BADR2. */
typedef enum
{
  CTRXCTRL = 0x60,
  IRCCTRL  = 0x6C
} BADR2OUT;

typedef enum
{
  IRCSTATUS = 0x6C,             ///< Status of encoders, it contains
  IRC0      = 0x70,             ///< Read a value from encoder 0
  IRC1      = 0x74,             ///< Read a value from encoder 1
  IRC2      = 0x78,             ///< Read a value from encoder 2
  IRC3      = 0x7C              ///< Read a value from encoder 3
} BADR2IN;


#define LOOP_AD_TIMEOUT 0xFFFF  ///< timeout per loop for endless AD conversion



/** write to memory mapped device word wise. */
static __inline void StoreWord(size_t Ptr, int Offset, __int16 value)
{
  ((volatile unsigned __int16 *)Ptr)[Offset/2] = value;
}


/** write to memory mapped device word wise through cache. */
static __inline void StoreCachedWord(size_t Ptr, int Offset, __int16 value, __int16 *CachedValue)
{
  if (*CachedValue != value)
    {
    *CachedValue = value;
    StoreWord(Ptr,Offset,value);
    }
}


/** read from memory mapped device word wise. */
static __inline __int16 GetWord(size_t Ptr, int Offset)
{
  return ((volatile unsigned __int16 *)Ptr)[Offset/2];
}


/** write to memory mapped device doubleword wise. */
static __inline void StoreDword(size_t Ptr, int Offset, __int32 value)
{
  ((volatile unsigned __int32 *)Ptr)[Offset/4] = value;
}


/** write to memory mapped device word wise through cache. */
static __inline void StoreCachedDword(size_t Ptr, int Offset, __int32 value, __int32 *CachedValue)
{
  if (*CachedValue != value)
    {
    *CachedValue = value;
    StoreDword(Ptr,Offset,value);
    }
}


/** read from memory mapped device double word wise. */
static __inline __int32 GetDword(size_t Ptr, int Offset)
{
  return ((volatile unsigned __int32 *)Ptr)[Offset/4];
}


/****************************************************************
 *                                                              *
 *                 DIGITAL INPUTS & OUTPUTS                     *
 *                                                              *
 ****************************************************************/


/** Get data from digital input. */
static int MF624DIRead(const DeviceRecord *DevRecord, unsigned channel)
{
  if (channel!=0) return 0;
  return GetWord(DevRecord->DrvRes.Resources.MemResources[1].Base,DIN);
}


/** Write data to digital output. */
static HUDAQSTATUS MF624DOWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned value)
{
  if (channel!=0) return HUDAQBADARG;
  StoreCachedWord(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, value,
                  & ((MF624_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
  return HUDAQSUCCESS;
}


/** Write one bit to digital output. */
static void MF624DOWriteBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value)
{
__int16 DoutWord;
  if (channel!=0) return;
  if (bit>=8) return;   //there are only 8 bits available in MF624

  DoutWord = ((MF624_Private *)(DevRecord->DrvRes.DriverData))->DoutCache;
  if (value) DoutWord |= (1<<bit);
       else DoutWord &= ~(1<<bit);

  StoreCachedWord(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, DoutWord,
                  & ((MF624_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
}


static void MF624DOWriteMultipleBits(const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value)
{
__int16 DoutWord;
  if (channel!=0) return;

  DoutWord = ((MF624_Private *)(DevRecord->DrvRes.DriverData))->DoutCache;
  DoutWord = (DoutWord & ~mask) | (value & mask);

  StoreCachedWord(DevRecord->DrvRes.Resources.MemResources[1].Base, DOUT, DoutWord,
                  & ((MF624_Private *)(DevRecord->DrvRes.DriverData))->DoutCache );
}


static double MF624DIOGetParameter(unsigned channel, HudaqParameter param)
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


static HUDAQSTATUS MF624AISetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF624_Private *Cache;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=AD_CHANNELS) return HUDAQBADARG;

  switch(param)
    {
    case HudaqAIUNITS:
          if (value!=0)
                Cache->AIOptions[channel] |= 1;
          else
                Cache->AIOptions[channel] &= ~1;
          return HUDAQSUCCESS;

    case HudaqAIRange:
          if ((int)value == 0) return HUDAQSUCCESS;   // only one range "0" -10:10V is supported
          return HUDAQBADARG;
    }

  return HUDAQNOTSUPPORTED;
}


static double MF624AIGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF624_Private *Cache;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=AD_CHANNELS) return WRONG_VALUE;

  switch(param)
    {
    case HudaqAIUNITS:
           return(Cache->AIOptions[channel] & 1);
    case HudaqAIRange:
           return 0;
    case HudaqAINUMCHANNELS:
	   return AD_CHANNELS;
    }

  return WRONG_VALUE;
}


/** Get data from analog input. */
static double MF624AIRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr0;
size_t Ptr1;
unsigned TimeoutCounter;
MF624_Private *Cache;

  if (channel>=AD_CHANNELS) return UNDEFINED_VALUE;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base;
  StoreCachedWord(Ptr1,ADCTRL,1<<channel,&Cache->ADCtrlCache);
  GetWord(Ptr1,ADSTART);	      /* Start A/D conversion - dummy read */
  Ptr0 = DevRecord->DrvRes.Resources.MemResources[0].Base;

  TimeoutCounter=0;
  while(GetDword(Ptr0,GPIOC) & GPIOC_EOLC)
    {
    if (TimeoutCounter++>LOOP_AD_TIMEOUT)
      return UNDEFINED_VALUE;           /*timeout*/
    }

  if ((Cache->AIOptions[channel]&1)==1)
    return GetWord(Ptr1,ADDATA);
  else
    return MF624_AD2VOLT(GetWord(Ptr1,ADDATA));
}


/** Read multiple data from analog input. */
static HUDAQSTATUS MF624AIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, double *values)
{
size_t Ptr0;
size_t Ptr1;
unsigned i;
unsigned __int16 Mask;
unsigned __int16 RawValues[8];
unsigned __int32 ReadCache;
int RetVal = HUDAQSUCCESS;

  if (number<=0 || channels==NULL || values==NULL) return HUDAQBADARG;

  Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base;
  Ptr0 = DevRecord->DrvRes.Resources.MemResources[0].Base;

  Mask=0;       /* Prepare list of channels to convert. */
  for(i=0;i<number;i++)
    {
    if (channels[i]<=7)
      Mask |= 1<<channels[i];
    }

  StoreCachedWord(Ptr1,ADCTRL,Mask,&(((MF624_Private *)(DevRecord->DrvRes.DriverData))->ADCtrlCache));
  i = GetWord(Ptr1,ADSTART);            /* Start A/D conversion */
  i=0;
  while(GetDword(Ptr0,GPIOC) & GPIOC_EOLC)
    {
    if (i++>LOOP_AD_TIMEOUT)
        return HUDAQFAILURE; /*timeout*/
    }

  i=0;
  while(i<=7)     /* Sort A/D values converted. */
    {
    if ( (Mask & (1<<i)) != 0)
      {
      Mask &= ~(1<<i);                     /* Cleanup mask flag. */
      if (Mask==0)                          /* Do we have only one last read? */
        RawValues[i] = GetWord(Ptr1,ADDATA);
      else
        {
        ReadCache = GetDword(Ptr1,ADDATA); /* Extract two values together from A/D FIFO */
        RawValues[i] = ReadCache;          /* feed the first value */

        while(i<=6)                        /* Look for second counterpart - compare to 6 because i is incremented below. */
          {
          i++;
          if ((Mask & (1<<i)) != 0)        /* Second counterpart has been found. */
            {
            Mask &= ~(1<<i);
            RawValues[i] = ReadCache>>16;  /* feed second value */
            break;
            }
          }
        }
      }
    i++;
    }

                        /* Store conversion results into output structure. */
  for(i=0;i<number;i++)
    {
    if (channels[i]<=7)
      {			/* When you switch different channel mode, be consistent with AIRead. */
      if ((((MF624_Private *)(DevRecord->DrvRes.DriverData))->AIOptions[channels[i]]&1)==1)
        values[i] = RawValues[channels[i]];
      else
        values[i] = MF624_AD2VOLT(RawValues[channels[i]]);
      }
    else
      {
      values[i] = UNDEFINED_VALUE;
      RetVal = HUDAQPARTIAL;
      }

    }
  return RetVal;
}


static double MF624AOGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF624_Private *Cache;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=DA_CHANNELS) return WRONG_VALUE;

  switch(param)
    {
    case HudaqAOUNITS:
           return(Cache->DA[channel].Options & 1);
           return HUDAQSUCCESS;
    case HudaqAORange:
           return 0;
    case HudaqAONUMCHANNELS:
	   return DA_CHANNELS;
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS MF624AOSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF624_Private *Cache;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=DA_CHANNELS) return HUDAQBADARG;

  switch(param)
    {
    case HudaqAOUNITS:
          if (value!=0)
                Cache->DA[channel].Options |= 1;
          else
                Cache->DA[channel].Options &= ~1;
          return HUDAQSUCCESS;

    case HudaqAORange:
          if ((int)value == 0) return HUDAQSUCCESS;   // only one range "0" -10:10V is supported
          return HUDAQBADARG;
    }
  return HUDAQNOTSUPPORTED;
}


/** Write data to analog output. */
static void MF624AOWrite(const DeviceRecord *DevRecord, unsigned channel, double value)
{
DaSetup *DAx;
  if (channel>=DA_CHANNELS) return;

  DAx = &((MF624_Private *)(DevRecord->DrvRes.DriverData))->DA[channel];
  StoreCachedWord(DevRecord->DrvRes.Resources.MemResources[1].Base,
      DADATA + 2*channel,
      ((DAx->Options & 1) == 1) ? (__int16)value : MF624_VOLT2DA(value),
      &DAx->Cache );
}


/** Write multiple data to analog output synchronized. */
static HUDAQSTATUS MF624AOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const double* values)
{
size_t Ptr0;
size_t Ptr1;
MF624_Private *Cache;
int RetVal = HUDAQSUCCESS;
unsigned __int16 Val2DAC;
unsigned char DirtyMask=0;

  if (number<=0 || channels==NULL || values==NULL) return HUDAQBADARG;

  Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base;
  Ptr0 = DevRecord->DrvRes.Resources.MemResources[0].Base;
  Cache = (MF624_Private *)(DevRecord->DrvRes.DriverData);

       /* Decide which channel(s) to write. */
  while(number-->0)
    {
    if (*channels>=DA_CHANNELS)			/* Channel is out of range. */
      RetVal = HUDAQPARTIAL;
    else
      {
      if ((Cache->DA[*channels].Options & 1) == 1) /* Use raw value? */
        Val2DAC = (unsigned __int16)(*values);
      else
        Val2DAC = (unsigned __int16)MF624_VOLT2DA(*values);

      if (Val2DAC != Cache->DA[*channels].Cache)
        {
        DirtyMask |= 1<<*channels;
        Cache->DA[*channels].Cache = Val2DAC;
	}
      }
    values++;
    channels++;
   }

	/* Write all dirty channels to DAC. */
  if (DirtyMask!=0)
    {
    StoreDword(Ptr0,GPIOC,GPIOC_BASE|GPIOC_DACEN|GPIOC_LDAC);/*enable DACen + Hold LDAC*/

    for(number=0;number<DA_CHANNELS;number+=2)
      {
      if ((DirtyMask & (3<<number)) == (3<<number))
        {
        DirtyMask &= ~(3<<number);
        StoreDword(Ptr1, DADATA + number*2, Cache->DA[number].Cache + 0x10000*Cache->DA[number+1].Cache);
        }
      }

    for(number=0;number<DA_CHANNELS;number++)
      {
      if (DirtyMask & (1<<number))
        {
        DirtyMask &= ~(1<<number);
        StoreWord(Ptr1, DADATA + number*2, Cache->DA[number].Cache);
        }
      }

    StoreDword(Ptr0,GPIOC,GPIOC_BASE|GPIOC_DACEN);        /*enable DACen*/
    }
return RetVal;
}


/****************************************************************
 *                                                              *
 *                          ENCODERS                            *
 *                                                              *
 ****************************************************************/


static double MF624EncGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF624_Private *Cache;
IRC_Control *pCTRi;
__int8 IrcValue[ENC_CHANNELS];

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return WRONG_VALUE;

  switch(param)
    {
    case HudaqEncRESETONREAD:
           return(Cache->EncOptions[channel] & IrcResetCounter);
    case HudaqEncFILTER:
           return(Cache->EncCache & (0x80<<(8*channel)));
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
           *(__int32 *)IrcValue = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, IRCSTATUS);
           return(IrcValue[channel]);
    case HudaqEncNUMCHANNELS:
           return ENC_CHANNELS;
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS MF624EncSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
size_t Ptr2;
MF624_Private *Cache;
__int8 IrcValue[ENC_CHANNELS];
IRC_Control *pCTRi;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return HUDAQBADARG;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  switch(param)
    {
    case HudaqEncRESETONREAD:
           if (value!=0)
             Cache->EncOptions[channel] |= IrcResetCounter;
           else
             Cache->EncOptions[channel] &= ~IrcResetCounter;
           return HUDAQSUCCESS;

    case HudaqEncFILTER:	// 0 is false, any nonzero value expands to true.
           if (value!=0)
             StoreCachedDword(Ptr2, IRCCTRL, Cache->EncCache | (0x80<<(8*channel)), &Cache->EncCache);
           else
             StoreCachedDword(Ptr2, IRCCTRL, Cache->EncCache & ~(0x80<<(8*channel)), &Cache->EncCache);
           return HUDAQSUCCESS;

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
    }

  return HUDAQNOTSUPPORTED;
}


static HUDAQSTATUS MF624EncReset(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr2;
MF624_Private *Cache;
__int8 IrcValue[ENC_CHANNELS];
IRC_Control *pCTRi;
char OldResetMode;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=ENC_CHANNELS) return HUDAQBADARG;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  *(unsigned __int32*)&IrcValue = Cache->EncCache;
  pCTRi = (IRC_Control *)&IrcValue[channel];

  OldResetMode = pCTRi->RESET_MODE; //store original reset mode
  pCTRi->RESET_MODE = 1;                               //reset encoder
  StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
  pCTRi->RESET_MODE = OldResetMode;                    //enable counting
  StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
  return HUDAQSUCCESS;
}


static int MF624EncRead(const DeviceRecord *DevRecord, unsigned channel)
{
IRC_Control *pCTRi;
size_t Ptr2;
__int8 IrcValue[ENC_CHANNELS];
__int32 RetVal;
MF624_Private *Cache;
char OldResetMode;

  if (channel>=ENC_CHANNELS) return 0;
  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  RetVal=GetDword(Ptr2, IRC0 + 4*channel);      // read a value from encoder

  if (Cache->EncOptions[channel] & IrcResetCounter) //RESET counter on every read
    {
    *(unsigned __int32*)&IrcValue = Cache->EncCache;
    pCTRi = (IRC_Control *)&IrcValue[channel];

    OldResetMode = pCTRi->RESET_MODE; //store original reset mode
    pCTRi->RESET_MODE = 1;                              //reset encoder
    StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
    pCTRi->RESET_MODE = OldResetMode;                   //enable counting
    StoreDword(Ptr2, IRCCTRL, *(unsigned __int32 *)&IrcValue);
    }

  return RetVal;
}


/****************************************************************
 *                                                              *
 *                COUNTERS PWM + Count + Step                   *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS MF624PWMWrite(const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle)
{
double T1,T2;
double T;
Counter *CacheCtrItem;
size_t Ptr2;
CounterControl CTC;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;

  CacheCtrItem = &((MF624_Private *)DevRecord->DrvRes.DriverData)->counter[channel];
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  if (dutycycle>=1)
    {
    if (CacheCtrItem->Mode != OUT_1)
      {
      CacheCtrItem->Mode = OUT_1;
      *(unsigned *)&CTC=0;
      CTC.OutputControl = 3;
      StoreCachedDword(Ptr2, 0x10*channel,*(unsigned __int32 *)&CTC, (__int32 *)&CacheCtrItem->CCache);
      }
    return HUDAQSUCCESS;
    }
  if (dutycycle<=0)
    {
    if (CacheCtrItem->Mode != OUT_0)
      {
      CacheCtrItem->Mode = OUT_0;
      *(unsigned *)&CTC=0;
      CTC.OutputControl = 2;
      StoreCachedDword(Ptr2, 0x10*channel,*(unsigned __int32 *)&CTC, (__int32 *)&CacheCtrItem->CCache);
      }
    return HUDAQSUCCESS;
    }

  if (frequency<=0) return HUDAQBADARG; //Allow duty cycle 0 and 1 for f=0.

    /* Prescaller is not set automatically, but when user sets it, calculate with a value given. */
  if (CacheCtrItem->Mode != PWM) CTC.ClockSource = HudaqCtrCLOCK50MHz;
  switch (CTC.ClockSource)
    {
    case HudaqCtrCLOCK100kHz:T = MASTERFREQUENCY/frequency/500.0;break;
    case HudaqCtrCLOCK1MHz:  T = MASTERFREQUENCY/frequency/50.0; break;
    case HudaqCtrCLOCK10MHz: T = MASTERFREQUENCY/frequency/5.0;  break;
    case HudaqCtrCLOCK50MHz:
    default:		     T = MASTERFREQUENCY/frequency;      break;
    }

  if (dutycycle<0.5)
    {
    T1 = max(1.0, min(T*dutycycle, (double)0xFFFFFFFF+1.0));
    T2 = max(1.0, min(T-T1, (double)0xFFFFFFFF+1.0));
    }
  else
    {
    T2 = max(1.0, min(T*(1-dutycycle), (double)0xFFFFFFFF+1.0));
    T1 = max(1.0, min(T-T2, (double)0xFFFFFFFF+1.0));
    }
                                      //sorry, but ((int)(T1+.5))-1 do not work for all values
  StoreCachedDword(Ptr2, 0x10*channel+4, (unsigned __int32)(T1-0.5), &CacheCtrItem->ACache);
  StoreCachedDword(Ptr2, 0x10*channel+8, (unsigned __int32)(T2-0.5), &CacheCtrItem->BCache);

  if (CacheCtrItem->Mode != PWM)
    {
    CacheCtrItem->Mode = PWM;

        // STOP counter
    StoreDword(Ptr2, CTRXCTRL,
       (GLOB_CTR_STOP | GLOB_CTR_LOAD | GLOB_CTR_RESET | GLOB_CTR_OUT_RESET)<<(6*channel) );

    *(unsigned *)&CTC=0;
    //CTC.ClockSource=0;	//50MHz is default.
    CTC.Repetition=1;
    CTC.LoadToggle=1;
    CTC.OutputToggle=1;

        // SET counter CWR
    StoreCachedDword(Ptr2, 0x10*channel,*(unsigned __int32 *)&CTC, (__int32 *)&CacheCtrItem->CCache);

       // START couner
    StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_START)<<(6*channel));
    }

  if (CacheCtrItem->CtrFixPWM!=0)  //Fix relatively big value change
    {
    T1 = (unsigned __int32)GetDword(Ptr2, 0x10*channel+4);
    if (T1 > T+CacheCtrItem->CtrFixPWM)
      {
      StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_LOAD)<<(6*channel));
      }
    }

  return HUDAQSUCCESS;
}


static HUDAQSTATUS MF624CtrReset(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr2;
Counter *CacheCtrItem;
CounterControl CTC;

  if (channel>=CTR_CHANNELS) return HUDAQBADARG;

  CacheCtrItem = &((MF624_Private *)DevRecord->DrvRes.DriverData)->counter[channel];
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  //if ((options & IrcNoSwitchMode)==0)
    {
    if (CacheCtrItem->Mode != Counting)
      {
      CacheCtrItem->Mode = Counting;

          // When counting down, 0 causes CE, thus load 0xFFFFFFFF
      //StoreCachedDword(Ptr2, 0x10*channel+4, 0xFFFFFFFF, &Cache->counter[channel].ACache);
          // When counting up, 0xFFFFFFFF causes CE, thus load 0
      StoreCachedDword(Ptr2, 0x10*channel+4, 0, &CacheCtrItem->ACache);
				// Initial CTR mode setup.
      *(unsigned *)&CTC=0;
      CTC.ClockSource=5;        //Rise IN
      CTC.Repetition=1;
      CTC.LoadToggle=0;
      CTC.Direction=1;          //Up
          // SET counter CWR
      StoreCachedDword(Ptr2, 0x10*channel,*(__int32 *)&CTC, (__int32 *)&CacheCtrItem->CCache);
      }
    }

       // RESET couner and START it
  StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_START|GLOB_CTR_RESET)<<(6*channel));
return HUDAQSUCCESS;
}


static double MF624CtrGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
Counter *pCacheCtrItem;

  if (channel>=CTR_CHANNELS) return WRONG_VALUE;
  pCacheCtrItem = &((MF624_Private *)DevRecord->DrvRes.DriverData)->counter[channel];

  switch(param)
    {
    case HudaqCtrDIRECTION:
           return(pCacheCtrItem->CCache.Direction);
    case HudaqCtrRESETONREAD:
           return(pCacheCtrItem->CtrResetCounter);
    case HudaqCtrREPETITION:
           return(pCacheCtrItem->CCache.Repetition);
    case HudaqCtrLOADTOGGLE:
	   return(pCacheCtrItem->CCache.LoadToggle);
    case HudaqCtrOUTTOGGLE:
           return(pCacheCtrItem->CCache.OutputToggle);
    case HudaqPwmCLOCKSOURCE:
    case HudaqCtrCLOCKSOURCE:
           return(pCacheCtrItem->CCache.ClockSource);
    case HudaqPwmOUTPUTCONTROL:
    case HudaqCtrOUTPUTCONTROL:
           return(pCacheCtrItem->CCache.OutputControl);
    case HudaqCtrTRIGSOURCE:
           return(pCacheCtrItem->CCache.TriggerSource);
    case HudaqCtrTRIGTYPE:
           return(pCacheCtrItem->CCache.TriggerType);
    case HudaqCtrRETRIGGER:
	   return(pCacheCtrItem->CCache.ReTrigger);
    case HudaqPwmGATESOURCE:
    case HudaqCtrGATESOURCE:
           return(pCacheCtrItem->CCache.GateSource);
    case HudaqPwmGATEPOLARITY:
    case HudaqCtrGATEPOLARITY:
           return(pCacheCtrItem->CCache.GatePolarity);
    case HudaqPwmFILTER:
    case HudaqCtrFILTER:
           return(pCacheCtrItem->CCache.Filter);

    case HudaqPwmNUMCHANNELS:
    case HudaqCtrNUMCHANNELS:
           return CTR_CHANNELS;

    case HudaqPwmTHRESHOLD:
           return(pCacheCtrItem->CtrFixPWM/MASTERFREQUENCY);

    }

  return WRONG_VALUE;
}


static HUDAQSTATUS MF624CtrSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
Counter *pCacheCtrItem;

  pCacheCtrItem = &((MF624_Private *)DevRecord->DrvRes.DriverData)->counter[channel];
  if (channel>=CTR_CHANNELS) return HUDAQBADARG;

  if (pCacheCtrItem->Mode != Counting)
    MF624CtrReset(DevRecord,channel);     // Enforce switching into a counter mode

  switch(param)
    {
    case HudaqCtrRESETONREAD:		//Feature is provided by SW layer only.
           pCacheCtrItem->CtrResetCounter = ((int)value==0) ? 0 : 1;
           return HUDAQSUCCESS;
    case HudaqCtrREPETITION:
           if (value>=2 || value<0) return HUDAQBADARG;
	   if (pCacheCtrItem->CCache.Repetition != (int)value)
	     {
             pCacheCtrItem->CCache.Repetition = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrLOADTOGGLE:
	   if (value>=2 || value<0) return HUDAQBADARG;
	   if (pCacheCtrItem->CCache.LoadToggle != (int)value)
	     {
             pCacheCtrItem->CCache.LoadToggle = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrOUTTOGGLE:
	   if (value>=2 || value<0) return HUDAQBADARG;
	   if (pCacheCtrItem->CCache.OutputToggle != (int)value)
	     {
             pCacheCtrItem->CCache.OutputToggle = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqPwmCLOCKSOURCE:
    case HudaqCtrCLOCKSOURCE:
           if (value>=16 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.ClockSource != (int)value)
             {
             pCacheCtrItem->CCache.ClockSource = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
             }
           return HUDAQSUCCESS;
    case HudaqPwmOUTPUTCONTROL:
    case HudaqCtrOUTPUTCONTROL:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.OutputControl != (int)value)
	     {
             pCacheCtrItem->CCache.OutputControl = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrDIRECTION:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.Direction != (int)value)
	     {
             pCacheCtrItem->CCache.Direction = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);

             StoreCachedDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x10*channel+4,
			(pCacheCtrItem->CCache.Direction==1) ? 0 : 0xFFFFFFFF,
                        &pCacheCtrItem->ACache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrTRIGSOURCE:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.TriggerSource != (int)value)
	     {
             pCacheCtrItem->CCache.TriggerSource = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrTRIGTYPE:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.TriggerType != (int)value)
	     {
             pCacheCtrItem->CCache.TriggerType = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqCtrRETRIGGER:
	   if (value>=2 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.ReTrigger != (int)value)
	     {
             pCacheCtrItem->CCache.ReTrigger = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqPwmGATESOURCE:
    case HudaqCtrGATESOURCE:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.GateSource != (int)value)
	     {
             pCacheCtrItem->CCache.GateSource = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqPwmGATEPOLARITY:
    case HudaqCtrGATEPOLARITY:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.GatePolarity != (int)value)
	     {
             pCacheCtrItem->CCache.GatePolarity = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;
    case HudaqPwmFILTER:
    case HudaqCtrFILTER:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCacheCtrItem->CCache.Filter != (int)value)
	     {
             pCacheCtrItem->CCache.Filter = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x10*channel,*(__int32 *)&pCacheCtrItem->CCache);
	     }
           return HUDAQSUCCESS;

    case HudaqPwmTHRESHOLD:
           pCacheCtrItem->CtrFixPWM = (value==0)? 0 : MASTERFREQUENCY/value;
           return HUDAQSUCCESS;
    }

  return HUDAQNOTSUPPORTED;
}


static int MF624CtrRead(const DeviceRecord *DevRecord, unsigned channel)
{
size_t Ptr2;
Counter *CacheCtrItem;
__int32 RetVal;

  if (channel>=CTR_CHANNELS) return 0;

  CacheCtrItem = &((MF624_Private *)DevRecord->DrvRes.DriverData)->counter[channel];
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  if (CacheCtrItem->Mode != Counting)
    {
    MF624CtrReset(DevRecord,channel);
    return 0;
    }

  RetVal = GetDword(Ptr2, 0x10*channel+4);

  if (CacheCtrItem->CtrResetCounter)        // Reset a counter after each read
    {
    StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_RESET)<<(6*channel));
    }

  return RetVal;
}


/****************************************************************************/
/** Stepout procedure makes a defined amount of steps
*****************************************************************************/
static HUDAQSTATUS MF624StepWrite(const DeviceRecord *DevRecord, unsigned channel, int position)
{
int digin;
int State=0;
size_t Ptr2;
long NewSteps;
MF624_Private *Cache;
CounterControl CTC;
unsigned __int32 N;
DWORD NewTime;
double NewFrequency;
Stepper *CacheStep;

  NewTime=timeGetTime();

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=STEP_CHANNELS) return HUDAQBADARG;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;
  CacheStep = &(Cache->step[channel]);

  if (Cache->counter[2*channel].Mode != StepperPWM)
    {           //check for the first time initialization of slave counter
    Cache->counter[2*channel].Mode = StepperPWM;

    *(unsigned *)&CTC=0;
    CTC.Repetition=1;
    CTC.OutputToggle=1;
    CTC.GateSource=3;           //OUT(n+1)
    CTC.GatePolarity=0;

      // SET counter CWR
    StoreCachedDword(Ptr2, 0x10*(2*channel),*(__int32 *)&CTC, (__int32 *)&Cache->counter[2*channel].CCache);

      //set a real frequency
    N = (unsigned __int32)(MASTERFREQUENCY / (2*CacheStep->Fmin));
    N = max(0,N-1);

    StoreCachedDword(Ptr2, 0x10*(2*channel)+4, N, &Cache->counter[2*channel].ACache);

    State|=1;
    }
  if (Cache->counter[2*channel+1].Mode != StepperMaster)
    {      //check for the first time initialization of master counter
    Cache->counter[2*channel+1].Mode = StepperMaster;

    *(unsigned *)&CTC=0;
    CTC.Repetition=0;
    CTC.OutputToggle=0;
    CTC.ClockSource=10;         //Fall N-1

    // SET counter CWR
    StoreCachedDword(Ptr2, 0x10*(2*channel+1),*(__int32 *)&CTC, (__int32 *)&Cache->counter[2*channel+1].CCache);

    State|=2;
    }

  if (State!=0)         //check for the first time initialization
    {
    CacheStep->Direction=0;
    CacheStep->LastPosition=0;
    CacheStep->ChainedPosition=0;
    CacheStep->Frequency=0;

    StoreDword(Ptr2, CTRXCTRL,
         (GLOB_CTR_STOP|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET)<<(6*(2*channel)) |    //reset PWM counter
         (GLOB_CTR_STOP|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET)<<(6*(2*channel+1)) ); //reload step NO counter
    }


                //********* check for end switches **********
  digin = MF624DIRead(DevRecord,0);
  if ( ((digin & (1<<(2*channel)))==0 && (CacheStep->Direction>0)) ||
    ( ((digin & (1<<(2*channel+1)))==0) && (CacheStep->Direction<0)) )  // end switch
    {
    StoreCachedDword(Ptr2, 0x10*(2*channel+1)+4, 0, &Cache->counter[2*channel+1].ACache); //fill A register with 0
    NewSteps = GetDword(Ptr2, 0x10*(2*channel+1)+4);            //read counter value
    if ((GetDword(Ptr2,0x10*(2*channel+1))&1) == 0)              //Is counter still running?
      NewSteps = 0;

    StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_STOP)<<(6*(2*channel)) |       //stop PWM counter
                               (GLOB_CTR_LOAD)<<(6*(2*channel+1)) );    //reload step NO counter

    CacheStep->LastPosition -= NewSteps * CacheStep->Direction;         //actualize real position

    CacheStep->Frequency = 0;
    CacheStep->Direction = 0;
    CacheStep->TimeStamp = NewTime;
    return HUDAQSUCCESS;
    }

                //***** actualize step count ******

  if (CacheStep->LastPosition!=position)
    {                                     //amount of steps to do
    if ( (State&3)!=0 ) //the first time initialization
      {
      NewSteps = 0;
      }
    else               // counter already runs
      {
      NewSteps = GetDword(Ptr2, 0x10*(2*channel+1)+4);
      if ((GetDword(Ptr2,0x10*(2*channel+1))&1) == 0)              //Is counter still running?
        NewSteps = 0;
      }

    if (NewSteps==0)
      {
      CacheStep->Direction=0;   //motor is already stopped.
      CacheStep->Frequency=0;
      }

          // CurrentPosition = LastPosition - |x|*Direction
          // CurrentPosition = NewPosition - NewSteps
          //   ===> NewSteps = NewPosition - LastPosition + |x|*Direction
    NewSteps = position - CacheStep->LastPosition + NewSteps*CacheStep->Direction;

    if (NewSteps>0)
      {
      if (CacheStep->Direction==-1)
        {       //direction has been reversed, calculate a false target
          //NewSteps = Fmin(channel)*Fmin(channel)+Frequency[channel]*Frequency[channel])/(2*Acc(channel)) - 1;
                // \todo   Study behaviour of this feature. It lets stepper to finish in oposite direction.
        goto TargetIsFixed;
        }
      CacheStep->LastPosition = position;   //actualize position cache
      CacheStep->Direction=1;
      StoreCachedDword(Ptr2, 0x10*(2*channel+1)+4, NewSteps, &Cache->counter[2*channel+1].ACache);
      State |= 4;       //reload & run counter
      }
    if (NewSteps<0)
      {
      if (CacheStep->Direction==1)         //if (Frequency[channel]>Fmin[channel])
        {       //direction has been reversed, calculate a false target
          //NewSteps = Fmin(channel)*Fmin(channel)+Frequency[channel]*Frequency[channel])/(2*Acc(channel)) - 1;
                        // \todo   Study behaviour of this feature. It lets stepper to finish in oposite direction.
        goto TargetIsFixed;
        }
      CacheStep->LastPosition = position;   //actualize position cache
      CacheStep->Direction=-1;
      StoreCachedDword(Ptr2, 0x10*(2*channel+1)+4, -NewSteps, &Cache->counter[2*channel+1].ACache);
      State |= 4;       //reload & run counter
      }
    if (NewSteps==0)
      {
      //CacheStep->LastPosition;	does not need to be actualised
      CacheStep->Direction=0;
      }
    }

TargetIsFixed:
  CacheStep->ChainedPosition = position;   // chained position makes sense only if direction is reversed

       //************ set frequency - (lastreq contains amount of remainning steps) ***********
  if (CacheStep->Acc<=0)
    CacheStep->Frequency=CacheStep->Fmax;        // full acceleration when accteleration is negative
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

    //set a real frequency
  NewFrequency = MASTERFREQUENCY / (2*CacheStep->Frequency);
  if (NewFrequency>0xFFFFFFFF) NewFrequency=0xFFFFFFFF;
  N = max(0,(unsigned __int32)NewFrequency-1);

  StoreCachedDword(Ptr2, 0x10*(2*channel)+4, N, &Cache->counter[2*channel].ACache);

    //write stepper direction to digital output
  if (CacheStep->Direction!=0)  // only when stepper is expected to be running
    MF624DOWriteBit(DevRecord,0,channel,CacheStep->Direction>0);

    //start counters
  if ( (State&4) != 0 )
    {
    StoreDword(Ptr2, CTRXCTRL, (GLOB_CTR_START)<<(6*(2*channel)) |      //stop PWM counter
                               (GLOB_CTR_LOAD|GLOB_CTR_START)<<(6*(2*channel+1)) );    //reload step NO counter
    }

  CacheStep->TimeStamp = NewTime;               //refresh timestamp
  return HUDAQSUCCESS;
}


static HUDAQSTATUS MF624StepSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
MF624_Private *Cache;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=STEP_CHANNELS) return HUDAQBADARG;

  if (Cache->counter[2*channel].Mode != StepperPWM || Cache->counter[2*channel+1].Mode != StepperMaster)
    MF624StepWrite(DevRecord,channel,0);     //call stepper initialization if it is not called before

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
    case HudaqStepCURRENTPOSITION:      // !!!!! Test value>MAXINT !!!!!
        Cache->step[channel].LastPosition =
           (int)value + Cache->step[channel].Direction * GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x10*(2*channel+1)+4);
        // CurrentPosition = LastPosition - Direction * AmountOfSteps
        //     NewPosition = NewLastPosition - Direction * AmountOfSteps
        // ==>  NewLastPosition = NewPosition + Direction * AmountOfSteps

        if ((GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base,0x10*(2*channel+1))&1) == 0) //Is counter still running or stopped?
           Cache->step[channel].LastPosition = (int)value;
        return HUDAQSUCCESS;
    }

return HUDAQNOTSUPPORTED;
}


static double MF624StepGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
MF624_Private *Cache;
size_t Ptr2;
int RetVal;

  Cache = (MF624_Private *)DevRecord->DrvRes.DriverData;
  if (channel>=STEP_CHANNELS) return WRONG_VALUE;

  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  if (Cache->counter[2*channel].Mode != StepperPWM || Cache->counter[2*channel+1].Mode != StepperMaster)
    MF624StepWrite(DevRecord,channel,0);     //call stepper initialization if it is not called before

  switch(param)
    {
    case HudaqStepFMIN:
        return(Cache->step[channel].Fmin);
    case HudaqStepFMAX:
        return(Cache->step[channel].Fmax);
    case HudaqStepACCELERATION:
        return(Cache->step[channel].Acc);
    case HudaqStepCURRENTPOSITION:      // CurrentPosition = LastPosition - Direction * AmountOfSteps
        return(
           Cache->step[channel].LastPosition -
           Cache->step[channel].Direction * GetDword(Ptr2, 0x10*(2*channel+1)+4));
    case HudaqStepREMAININGSTEPS:
        RetVal = GetDword(Ptr2, 0x10*(2*channel+1)+4);
        if ((GetDword(Ptr2,0x10*(2*channel+1))&1) == 0)              //Is counter still running or stopped?
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
 *                    MF625 specific code                       *
 *                                                              *
 ****************************************************************/

/** MF625 has only one internal counter. Give a chance to handle it. */
static int MF625CtrRead(const DeviceRecord *DevRecord, unsigned channel)
{
  return MF624CtrRead(DevRecord,channel+4);
}


static HUDAQSTATUS MF625CtrReset(const DeviceRecord *DevRecord, unsigned channel)
{
  return MF624CtrReset(DevRecord,channel+4);
}


static double MF625CtrGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
  if (channel>=1) return WRONG_VALUE;

  switch(param)
    {
    case HudaqCtrNUMCHANNELS:
           return 1;
    default: return MF624CtrGetParameter(DevRecord,channel+4,param);
    }
  return WRONG_VALUE;
}


static HUDAQSTATUS MF625CtrSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
  if (channel>=1) return HUDAQBADARG;
  return MF624CtrSetParameter(DevRecord,channel+4,param,value);
}


////////////////////////////////////////////////////////////////////////////

#define MAX24BITUINT 0xFFFFFF	///< Maximal range of MF625 counters.

/** Internal PWM procedure is used from both 3 phase and 6 phase PWM. */
static HUDAQSTATUS iMF625PWMMultiphaseWrite(const DeviceRecord *DevRecord, unsigned NewDivider, const unsigned *CompareVals)
{
typedef enum
{
  DirtyDIVIDER = 0x100,
  PwmSTOPPED =   0x200,
  AnotherMODE =  0x400,
  NeedRELOAD =   0x800
} DirtyFlags;
double T, CounterValue=-1;
PWMMultiple *pCachePWM;
size_t Ptr2;
unsigned dirty = 0;
unsigned i;
int Timeout = 120;	//Timeout for waiting to edge

  pCachePWM = &((MF624_Private *)DevRecord->DrvRes.DriverData)->pwm;

  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

    /* Counter prepare phase */
  if (pCachePWM->Mode==Phase3PWM || pCachePWM->Mode==Phase6PWM)
    {	/* Counter runs in a right mode, check whether update is pending. */
    *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);

    while (pCachePWM->CTR.UpdatePending && pCachePWM->CTR.Running && !pCachePWM->CTR.Transparent)
      { /* Yes, exit loop when time is sufficient or update pending flag is cleared (or Timeout occurs :-(() */
      CounterValue = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x4);
      if (pCachePWM->CTR.UpDown)
        T = 2.0*pCachePWM->Divider - CounterValue; //when going up, enlarge time.
      else
	T = CounterValue;

      switch (pCachePWM->CTR.ClockSource)	// convert T to time units
        {
        case HudaqCtrCLOCK100kHz:T *= 500.0/MASTERFREQUENCY;
			         break;
	case HudaqCtrCLOCK1MHz:  T *= 50.0/MASTERFREQUENCY;
			         break;
	case HudaqCtrCLOCK10MHz: T *= 5.0/MASTERFREQUENCY;
			         break;
        case HudaqCtrCLOCK50MHz: T *= 1.0/MASTERFREQUENCY;
			         break;
	default:		 T *= 1.0/MASTERFREQUENCY;
				 Timeout = -1;	//Don't wait for external unknown clock.
			         break;
	}
      if (T>=8e-6) break;	 // more than 8us is sufficient enough for feeding 7 registers.

      if (Timeout-- < 0) break;
      *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);
      }

    if (pCachePWM->CTR.Running==0) dirty |= PwmSTOPPED;  //when dirty>0, counter will be started t the end of this procedure
    *(__int32 *)&(pCachePWM->CTR) &= ~MF625_COMMANDS; //clear all command positions
    }
  else  // A counter is switched in another mode, stop it.
    {
    *(int *)&(pCachePWM->CTR) = 0;		//reset internal register
    StoreDword(Ptr2, 0x0, *(__int32*)(&pCachePWM->CTR) |MF625_STOP|MF625_RESET);
    dirty |= AnotherMODE;
    }

    /* Write frequency divider */
  if (NewDivider != pCachePWM->Divider)
    {
    pCachePWM->Divider = NewDivider;
    StoreDword(Ptr2, 4, pCachePWM->Divider);
    dirty |= DirtyDIVIDER;
    }

    /* Write comparator values for particular channels. */
  for(i=0; i<PWM_625_SUBCHANNELS; i++)
    {
    if (CompareVals[i]!=pCachePWM->Comparators[i])
      {
      StoreDword(Ptr2, 0x10+4*i, CompareVals[i]);
      pCachePWM->Comparators[i] = CompareVals[i];
      dirty |= 1<<i;
      }
    }

    /* Are we doing initialization? */
  if (pCachePWM->Mode!=Phase3PWM && pCachePWM->Mode!=Phase6PWM)
    {
    if (pCachePWM->Mode==Phase3PWM_prep)
      {
      // pCachePWM->CTR.Inversions = 0x38;  //111000b  - switch on 0
      pCachePWM->CTR.Inversions = 0x07;	    //000111b  - inverted approach (Jirka's hack)
      pCachePWM->Mode=Phase3PWM;
      }
    else
      {
      pCachePWM->CTR.Inversions = 0;	// don't invert anything
      pCachePWM->Mode=Phase6PWM;
      }
    pCachePWM->CTR.Filter = 1; 		// Turn on a filter, for safety. User could turn a filter off by ::SetParameter.

    pCachePWM->CTR.OutputControl = HudaqCtrOUTPUTNORMAL; // 00b - enable direct output
    StoreDword(Ptr2, 0x0, *(__int32*)(&pCachePWM->CTR) |MF625_START|MF625_RESET|MF625_FORCE_UPDATE);
    dirty = 0;			        // clean up dirty flag
    }

    /* Masking dirty flag eliminates unnecesary write when transparent mode is on. */
  if (pCachePWM->CTR.Transparent) dirty &= ~0x1FF;  // mask divider and all phase channels

  if (pCachePWM->CtrFixPWM!=0 && (dirty & 0x100))  //Fix relatively big value change (only when divider changed)
    {
    if (CounterValue==-1) CounterValue = (unsigned __int32)GetDword(Ptr2, 0x4);
    if (CounterValue > pCachePWM->Divider+pCachePWM->CtrFixPWM)
      		dirty |= NeedRELOAD;
    }

    /* When dirty, shedule update on down->up counter switch. */
  if (dirty>0)
    {
    StoreDword(Ptr2, 0x0, *(__int32*)(&pCachePWM->CTR) |MF625_SHEDULE_UPDATE|MF625_START
		                      | (((dirty&NeedRELOAD) ==0) ? 0 : MF625_LOAD) );
    }

  return HUDAQSUCCESS;
}


static HUDAQSTATUS MF625PWMMultiphaseWrite(const DeviceRecord *DevRecord, unsigned channel, unsigned phasenum, const unsigned *phases, double frequency, const double *duties)
{
unsigned CompareVals[6];
unsigned i;
PWMMultiple *pCachePWM;
double T;
unsigned DefaultChannels[PWM_625_SUBCHANNELS] = {0,1,2,3,4,5};

  if (channel>=1) return HUDAQBADARG;
  pCachePWM = &((MF624_Private *)DevRecord->DrvRes.DriverData)->pwm;

  if (phases==NULL)
    {
    if (phasenum>PWM_625_SUBCHANNELS) return HUDAQBADARG;
    phases=DefaultChannels;
    }

  if (frequency<0) return HUDAQBADARG;

   /* Check for valid channel numbers. */
  for (i=0; i<phasenum; i++)
    {
    if (phases[i]>=PWM_625_SUBCHANNELS) return HUDAQBADARG;
       //Allow duty cycle 0 or 1 for f=0.
    if (frequency==0 && (phases[i]>0 && phases[i]<1)) return HUDAQBADARG;
    }

    /* Feed old values. */
  for(i=0;i<PWM_625_SUBCHANNELS;i++)
    {
    CompareVals[i] = pCachePWM->Comparators[i];
    }

  if (pCachePWM->Mode!=Phase6PWM)
    {
    pCachePWM->Mode=Phase6PWM_prep;
    pCachePWM->CTR.ClockSource=HudaqCtrCLOCK50MHz;
    }

  /* Calculate with prescaller. It is not set automatically, but user is allowed to set it when needed. */
  switch (pCachePWM->CTR.ClockSource)
    {
    case HudaqCtrCLOCK100kHz:T = MASTERFREQUENCY/500.0;break;
    case HudaqCtrCLOCK1MHz:  T = MASTERFREQUENCY/50.0; break;
    case HudaqCtrCLOCK10MHz: T = MASTERFREQUENCY/5.0;  break;
    case HudaqCtrCLOCK50MHz: T = MASTERFREQUENCY;      break;
		// When external clock is used, frequency means division ratio.
    default:		     T = 1.0;
    }

  if (frequency == 0) T = MAX24BITUINT-2;
  else {
       T = max((T/frequency/2.0) - 1.5, 0.0);
       T = min(T,(double)MAX24BITUINT);
       }

  while (phasenum>0)
    {  /* There is intentionally a value >(Divider+1), because it generates true 1 on output. */
    CompareVals[*phases] = (unsigned) min( max(*duties * ((unsigned)T+2), 0.0), (double)MAX24BITUINT );

    phases++;	/* Toss away these arrays. */
    duties++;
    phasenum--;
    }

  return iMF625PWMMultiphaseWrite(DevRecord, (unsigned)T, CompareVals);
}


HUDAQSTATUS MF625PWM3Write(const DeviceRecord *DevRecord, unsigned channel, double frequency, double duty1, double duty2, double duty3)
{
PWMMultiple *pCachePWM;
int DeadBandTicks;
double T;
unsigned CompareVals[6];

  if (channel>=1) return HUDAQBADARG;
  pCachePWM = &((MF624_Private *)DevRecord->DrvRes.DriverData)->pwm;

  if (frequency<=0)
    {
    if (frequency<0) return HUDAQBADARG;
       		//Allow duty cycle 0 or 1 for f=0.
    if ((duty1>0 && duty1<1) || (duty2>0 && duty2<1) || (duty3>0 && duty3<1))
      return HUDAQBADARG;
    }

/* Here is a small hack introduced by Jirka Sehnal. The key idea is to invert duty,
   deadband and output polarity together. It behaves same and it also guarantee symetrical
   pulses around counter value 'n'. MF625 flips on zero passing. After this hack it would
   look like it flips in n passing. */
  duty1 = 1 - duty1;
  duty2 = 1 - duty2;
  duty3 = 1 - duty3;


  if (pCachePWM->Mode!=Phase3PWM)
    {
    pCachePWM->Mode=Phase3PWM_prep;
    pCachePWM->CTR.ClockSource=HudaqCtrCLOCK50MHz;
    }

  /* Calculate with prescaller. It is not set automatically, but user is allowed to set it when needed. */
  switch (pCachePWM->CTR.ClockSource)
    {
    case HudaqCtrCLOCK100kHz:T = MASTERFREQUENCY/500.0;break;
    case HudaqCtrCLOCK1MHz:  T = MASTERFREQUENCY/50.0; break;
    case HudaqCtrCLOCK10MHz: T = MASTERFREQUENCY/5.0;  break;
    case HudaqCtrCLOCK50MHz: T = MASTERFREQUENCY;      break;
		// When external clock is used, frequency means division ratio.
    default:		     T = 1.0;
    }

  DeadBandTicks = (int)(T*pCachePWM->DeadBand + 0.5);

  if (frequency == 0) T = MAX24BITUINT-2;
  else {
       T = max((T/frequency/2.0) - 1.5, 0.0);
       T = min(T,(double)MAX24BITUINT);
       }

	//A differency between Phases[0]-Phases[3] is guaranteed
  CompareVals[0] = (unsigned)min( max(((int)T+2)*duty1 + DeadBandTicks/2.0, 0.0), (double)MAX24BITUINT);
  CompareVals[3] = (unsigned)min( max((int)CompareVals[0]-DeadBandTicks, 0), MAX24BITUINT);

  CompareVals[1] = (unsigned)min( max(((int)T+2)*duty2 + DeadBandTicks/2.0, 0.0), (double)MAX24BITUINT);
  CompareVals[4] = (unsigned)min( max((int)CompareVals[1]-DeadBandTicks, 0), MAX24BITUINT);

  CompareVals[2] = (unsigned)min( max(((int)T+2)*duty3 + DeadBandTicks/2.0, 0.0), (double)MAX24BITUINT);
  CompareVals[5] = (unsigned)min( max((int)CompareVals[2]-DeadBandTicks, 0), MAX24BITUINT);

  return iMF625PWMMultiphaseWrite(DevRecord, (unsigned)T, CompareVals);
}


static HUDAQSTATUS MF625PWMWrite(const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle)
{
  if (channel>=1) return HUDAQBADARG;
  return MF625PWMMultiphaseWrite(DevRecord, 0, 1, &channel, frequency, &dutycycle);
}


static double MF625PwmGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
PWMMultiple *pCachePWM;
int retval;

  if (channel>=PWM_625_CHANNELS) return WRONG_VALUE;
  pCachePWM = &((MF624_Private *)DevRecord->DrvRes.DriverData)->pwm;

  switch(param)
    {
    case HudaqPwmNUMCHANNELS:
           return PWM_625_CHANNELS;
    case HudaqPwmDEADBAND:
	   return pCachePWM->DeadBand;
    case HudaqPwmOUTPUTUDCONTROL:
           return pCachePWM->CTR.OutputUDCtrl;
    case HudaqPwmOUTPUTCONTROL:
           return pCachePWM->CTR.OutputControl;
    case HudaqPwmCLOCKSOURCE:
          return pCachePWM->CTR.ClockSource;
    case HudaqPwmFILTER:
          return pCachePWM->CTR.Filter;
    case HudaqPwmGATESOURCE:
	  return pCachePWM->CTR.GateSource;
    case HudaqPwmTRANSPARENT:
          return pCachePWM->CTR.Transparent;
    case HudaqPwmEMERGENCY:
          return pCachePWM->CTR.Emergency;
    case HudaqPwmGATEPOLARITY:
	  return pCachePWM->CTR.GatePolarity;
    case HudaqPwmINPUT:
          *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);
          retval = pCachePWM->CTR.InputWire;
	  *(__int32 *)&(pCachePWM->CTR) &= ~MF625_COMMANDS; //clear all command positions
          return retval;
    case HudaqPwmPHASES:
	  *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);
          retval = pCachePWM->CTR.PhaseOut;
	  *(__int32 *)&(pCachePWM->CTR) &= ~MF625_COMMANDS; //clear all command positions
          return retval;
    case HudaqPwmUPDOWN:
	  *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);
          retval = pCachePWM->CTR.UpDown;
	  *(__int32 *)&(pCachePWM->CTR) &= ~MF625_COMMANDS; //clear all command positions
          return retval;
    case HudaqPwmINVERSIONS:
          *(__int32 *)&(pCachePWM->CTR) = GetDword(DevRecord->DrvRes.Resources.MemResources[2].Base, 0x0);
          retval = pCachePWM->CTR.Inversions;
	  *(__int32 *)&(pCachePWM->CTR) &= ~MF625_COMMANDS; //clear all command positions
          return retval;
    case HudaqPwmTHRESHOLD:
          return(pCachePWM->CtrFixPWM/MASTERFREQUENCY);
    }

  return WRONG_VALUE;
}


static HUDAQSTATUS MF625PwmSetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
PWMMultiple *pCachePWM;
size_t Ptr2;

  if (channel>=PWM_625_CHANNELS) return HUDAQBADARG;
  pCachePWM = &((MF624_Private *)DevRecord->DrvRes.DriverData)->pwm;
  Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

  switch(param)
    {
    case HudaqPwmDEADBAND:
	   if (value<0 || value>1e-3) return HUDAQBADARG; //allow range <0;1ms>
	   pCachePWM->DeadBand = value;
	   return HUDAQSUCCESS;

    case HudaqPwmFILTER:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.Filter != (int)value)
	     {
             pCachePWM->CTR.Filter = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

    case HudaqPwmOUTPUTCONTROL:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.OutputControl != (int)value)
	     {
             pCachePWM->CTR.OutputControl = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

    case HudaqPwmOUTPUTUDCONTROL:
	   if (value>=4 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.OutputUDCtrl != (int)value)
	     {
             pCachePWM->CTR.OutputUDCtrl = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

    case HudaqPwmCLOCKSOURCE:
           if (value>=16 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.ClockSource != (int)value)
             {
             pCachePWM->CTR.ClockSource = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
             }
           return HUDAQSUCCESS;

    case HudaqPwmGATEPOLARITY:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.GatePolarity != (int)value)
	     {
             pCachePWM->CTR.GatePolarity = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

    case HudaqPwmGATESOURCE:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.GateSource != (int)value)
	     {
             pCachePWM->CTR.GateSource = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

     case HudaqPwmTRANSPARENT:
           if (value>=2 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.Transparent != (int)value)
	     {
             pCachePWM->CTR.Transparent = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

      case HudaqPwmEMERGENCY:
           if (value>=4 || value<0) return HUDAQBADARG;
           if (pCachePWM->CTR.Emergency != (int)value)
	     {
             pCachePWM->CTR.Emergency = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

      case HudaqPwmINVERSIONS:
           if (value>0x3F || value<0) return HUDAQBADARG;  //6 bits only
           if (pCachePWM->CTR.Inversions != (int)value)
	     {
             pCachePWM->CTR.Inversions = (int)value;
             StoreDword(DevRecord->DrvRes.Resources.MemResources[2].Base,
                        0x0,*(__int32 *)&pCachePWM->CTR);
	     }
           return HUDAQSUCCESS;

       case HudaqPwmTHRESHOLD:
           pCachePWM->CtrFixPWM = (value==0)? 0 : MASTERFREQUENCY/value;
           return HUDAQSUCCESS;

    }

  return HUDAQNOTSUPPORTED;
}


/****************************************************************
 *                                                              *
 *                    GET/SET PARAMETERS                        *
 *                                                              *
 ****************************************************************/

static HUDAQSTATUS MF624SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  //case HudaqDI:
  //case HudaqDO:
  case HudaqAI:  return MF624AISetParameter(DevRecord,channel,param,value);
  case HudaqAO:  return MF624AOSetParameter(DevRecord,channel,param,value);
  case HudaqEnc: return MF624EncSetParameter(DevRecord,channel,param,value);
  case HudaqPWM:
  case HudaqCtr: return MF624CtrSetParameter(DevRecord,channel,param,value);
  case HudaqStep:return MF624StepSetParameter(DevRecord,channel,param,value);
  }
return HUDAQNOTSUPPORTED;
}


static HUDAQSTATUS MF625SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  //case HudaqDI:
  //case HudaqDO:
  case HudaqAI:  return MF624AISetParameter(DevRecord,channel,param,value);
  case HudaqAO:  return MF624AOSetParameter(DevRecord,channel,param,value);
  case HudaqEnc: return MF624EncSetParameter(DevRecord,channel,param,value);
  case HudaqPWM: return MF625PwmSetParameter(DevRecord,channel,param,value);
  case HudaqCtr: return MF625CtrSetParameter(DevRecord,channel,param,value);
  //case HudaqStep:
  }
return HUDAQNOTSUPPORTED;
}


static HUDAQSTATUS AD622SetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value)
{
switch(param & HudaqSubsystemMASK)
  {
  //case HudaqDI:
  //case HudaqDO:
  case HudaqAI:  return MF624AISetParameter(DevRecord,channel,param,value);
  case HudaqAO:  return MF624AOSetParameter(DevRecord,channel,param,value);
  //case HudaqEnc:
  //case HudaqCtr:
  //case HudaqStep:
  }
return HUDAQNOTSUPPORTED;
}



static double MF624GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return MF624DIOGetParameter(channel,param);
  case HudaqAI:  return MF624AIGetParameter(DevRecord,channel,param);
  case HudaqAO:  return MF624AOGetParameter(DevRecord,channel,param);
  case HudaqEnc: return MF624EncGetParameter(DevRecord,channel,param);
  case HudaqPWM:
  case HudaqCtr: return MF624CtrGetParameter(DevRecord,channel,param);
  case HudaqStep:return MF624StepGetParameter(DevRecord,channel,param);
  case HudaqIRQ: return ((MF624_Private *)DevRecord->DrvRes.DriverData)->Hdr.IRQcounter;
  }
return WRONG_VALUE;
}


static double MF625GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return MF624DIOGetParameter(channel,param);
  case HudaqAI:  return MF624AIGetParameter(DevRecord,channel,param);
  case HudaqAO:  return MF624AOGetParameter(DevRecord,channel,param);
  case HudaqEnc: return MF624EncGetParameter(DevRecord,channel,param);
  case HudaqPWM: return MF625PwmGetParameter(DevRecord,channel,param);
  case HudaqCtr: return MF625CtrGetParameter(DevRecord,channel,param);
  //case HudaqStep:
  }
return WRONG_VALUE;
}


static double AD622GetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
  case HudaqDI:
  case HudaqDO:  return MF624DIOGetParameter(channel,param);
  case HudaqAI:  return MF624AIGetParameter(DevRecord,channel,param);
  case HudaqAO:  return MF624AOGetParameter(DevRecord,channel,param);
  //case HudaqEnc:
  //case HudaqPWM:
  //case HudaqCtr:
  //case HudaqStep:
  }
return WRONG_VALUE;
}


static const HudaqRange *MF624QueryRange(const DeviceRecord *DevRecord, HudaqSubsystem S, unsigned item)
{
  if (item>=1) return NULL;
  switch(S & HudaqSubsystemMASK)
    {
    case HudaqAI:  return &MF624RangeAIO;
    case HudaqAO:  return &MF624RangeAIO;
    }
  return NULL;
}


/****************************************************************
 *                                                              *
 *                       INITIALIZATION                         *
 *                                                              *
 ****************************************************************/

/** Initialize subset of hardware that is supported by both MF624 & AD622. */
static void InitAiAoDiDo(size_t Ptr0, size_t Ptr1, MF624_Private *Cache)
{
int i;

    /* Initialization of AD's */
    Cache->ADCtrlCache = 0;	  /* 0 in channel selector is inocent. First real cycle will use nonzero value. */
    for(i=0;i<AD_CHANNELS;i++)
      {
      Cache->AIOptions[i] = 0;
      }

    /* Initialization of DA's */
    for(i=0;i<DA_CHANNELS;i++)    /*First clear DAC registers and then allow GPIOC_DACEN to avoid hazard pulse.*/
      {
      StoreWord(Ptr1,DADATA+2*i,0x2000);
      Cache->DA[i].Cache = 0x2000;
      Cache->DA[i].Options = 0;
      }
    StoreDword(Ptr0, GPIOC, GPIOC_BASE|GPIOC_DACEN);  //enable DACen

    /* Initialization of digital outputs */
    Cache->DoutCache = 0;
    StoreWord(Ptr1, DOUT, 0);
}



/** Internal initialization function that should be called from HudaqOpenDevice
   * \todo Exclusive access is only partially implemented. */
static int Init624(DeviceRecord *DevRecord, int IniOptions)
{
size_t Ptr1, Ptr2;
int i;
MF624_Private *Cache;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<3) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL)
    {	/*NO CACHE FOUND, cache==NULL*/
    return -3;
    }
  if (DevRecord->DrvRes.DriverDataSize<sizeof(MF624_Private)) return -4;

  DevRecord->pCT = &CT624;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {                           //initialize hardware only if no other application is running
    Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base;
    Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

      /* Initialization of AD's, DA's, DI and DO. */
    InitAiAoDiDo(DevRecord->DrvRes.Resources.MemResources[0].Base,Ptr1,Cache);

      /* Initialization of encoders */
    Cache->EncCache=0;
    StoreDword(Ptr2,IRCCTRL,0x10101010); //All 4 Encoder CTRs are RESET to 0
    StoreDword(Ptr2,IRCCTRL,0);          //Set also their CWR to 0
    for(i=0;i<ENC_CHANNELS;i++)
      {
      Cache->EncOptions[i]=0;
      }

      /* Initialization of counters */
    for(i=0;i<CTR_CHANNELS;i++)
      {
      Cache->counter[i].Mode = OUT_0;
      *((__int32 *)&(Cache->counter[i].CCache)) = 0;
      Cache->counter[i].CCache.OutputControl = 2;       //permanent 0
      StoreDword(Ptr2, 0x10*i, *(__int32 *)&(Cache->counter[i].CCache)); // write CTR mode
      StoreDword(Ptr2, 0x10*i+4, 0);    // clear A register
      Cache->counter[i].ACache = 0;
      StoreDword(Ptr2, 0x10*i+8, 0);    // clear B register
      Cache->counter[i].BCache = 0;
      Cache->counter[i].CtrResetCounter = 0;
      Cache->counter[i].CtrFixPWM = MASTERFREQUENCY/1000.0;  //fix time difference bigger than 1ms
      }

    StoreDword(Ptr2, CTRXCTRL,           // stop and reset all counters
            GLOB_CTR_BASE0*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) |
            GLOB_CTR_BASE1*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) |
            GLOB_CTR_BASE2*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) |
            GLOB_CTR_BASE3*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) |
            GLOB_CTR_BASE4*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) );

      /* Initialization of stepper motors courtesy */
    for(i=0;i<2;i++)
      {
      Cache->step[i].LastPosition = 0;
      Cache->step[i].ChainedPosition = 0;
      Cache->step[i].Direction = 0;

      Cache->step[i].Fmin = 1000;
      Cache->step[i].Fmax = 5000;
      Cache->step[i].Acc = 1000;
      }

    }
  //else
  //  {
  //  if (exclusive) return -5;  // exclusive access failed, resources.c calls HudaqCloseDevice
  //  }

return 1; //success
}


/** Internal cleanup procedure for MF624 & MF625 & AD622 */
static void Done624(DeviceRecord *DevRecord)
{
  if (DevRecord==NULL) return;

  DevRecord->pCT = &CtDummy;
}


const CallTable CT624 =
    {
    "MF624", 0x186C, 0x0624,
    Init624,
    Done624,

    MF624SetParameter,
    MF624GetParameter,
    MF624QueryRange,

        // INITIALIZE DI callers
    MF624DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    MF624DOWrite,
    MF624DOWriteBit,
    MF624DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    MF624AIRead,
    MF624AIReadMultiple,
        // INITIALIZE AO callers
    MF624AOWrite,
    MF624AOWriteMultiple,
        // INITIALIZE Enc callers
    MF624EncRead,
    MF624EncReset,
        // INITIALIZE Ctr callers
    MF624CtrRead,
    MF624CtrReset,
        // INITIALIZE PWM callers
    MF624PWMWrite,
    NULL,
    NULL,
        // INITIALIZE Step callers
    MF624StepWrite,
    };


/** Internal initialization function that should be called from HudaqOpenDevice
   * \todo Exclusive access is only partially implemented. */
static int Init625(DeviceRecord *DevRecord, int IniOptions)
{
size_t Ptr1, Ptr2;
int i;
MF624_Private *Cache;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<3) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL)
    {
    //printf("\nNO CACHE FOUND, cache==NULL!");
    return -3;
    }
  if (DevRecord->DrvRes.DriverDataSize<sizeof(MF624_Private)) return -4;

  DevRecord->pCT = &CT625;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {                           //initialize hardware only if no other application is running
    Ptr1 = DevRecord->DrvRes.Resources.MemResources[1].Base;
    Ptr2 = DevRecord->DrvRes.Resources.MemResources[2].Base;

      /* Initialization of AD's, DA's, DI and DO. */
    InitAiAoDiDo(DevRecord->DrvRes.Resources.MemResources[0].Base,Ptr1,Cache);

      /* Initialization of encoders */
    Cache->EncCache=0;
    StoreDword(Ptr2,IRCCTRL,0x10101010); //All 4 Encoder CTRs are RESET to 0
    StoreDword(Ptr2,IRCCTRL,0);          //Set also their CWR to 0
    for(i=0;i<ENC_CHANNELS;i++)
      {
      Cache->EncOptions[i]=0;
      }

      /* Initialization of counters, MF625 supports only counter 4 */
    for(i=4;i<CTR_CHANNELS;i++)
      {
      Cache->counter[i].Mode = OUT_0;
      *((__int32 *)&(Cache->counter[i].CCache)) = 0;
      Cache->counter[i].CCache.OutputControl = 2;       //permanent 0
      StoreDword(Ptr2, 0x10*i, *(__int32 *)&(Cache->counter[i].CCache)); // write CTR mode
      StoreDword(Ptr2, 0x10*i+4, 0);    // clear A register
      Cache->counter[i].ACache = 0;
      StoreDword(Ptr2, 0x10*i+8, 0);    // clear B register
      Cache->counter[i].BCache = 0;
      Cache->counter[i].CtrResetCounter = 0;
      Cache->counter[i].CtrFixPWM = MASTERFREQUENCY/1000.0;  //fix time difference bigger than 1ms
      }

    StoreDword(Ptr2, CTRXCTRL,           // stop and reset internal counter
            GLOB_CTR_BASE4*(GLOB_CTR_STOP|GLOB_CTR_LOAD|GLOB_CTR_RESET|GLOB_CTR_OUT_RESET) );

      /* Initialization of specialised PWM counter. */
    Cache->pwm.DeadBand = 0;
    Cache->pwm.CtrFixPWM = 0;
    Cache->pwm.Mode = OUT_0_ALL; 	// start in the mode all zeros

    Cache->pwm.Divider = 0;
    StoreDword(Ptr2, 0x4, 0);

    for(i=0;i<PWM_625_SUBCHANNELS;i++)
      {
      Cache->pwm.Comparators[i] = 0;
      StoreDword(Ptr2, 0x10+4*i, 0);
      }

    *(__int32*)(&Cache->pwm.CTR) = 0;
    Cache->pwm.CTR.OutputControl = 2;	// 10b - fix output on 0
    StoreDword(Ptr2, 0x0, *(__int32*)(&Cache->pwm.CTR) |
                      MF625_STOP|MF625_RESET|MF625_CANCEL_SHEDULE|MF625_FORCE_UPDATE|MF625_FORCE_UP);
    }

return 1; //success
}

const CallTable CT625 =
    {
    "MF625", 0x186C, 0x0625,
    Init625,
    Done624,

    MF625SetParameter,
    MF625GetParameter,
    MF624QueryRange,

        // INITIALIZE DI callers
    MF624DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    MF624DOWrite,
    MF624DOWriteBit,
    MF624DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    MF624AIRead,
    MF624AIReadMultiple,
        // INITIALIZE AO callers
    MF624AOWrite,
    MF624AOWriteMultiple,
        // INITIALIZE Enc callers
    MF624EncRead,
    MF624EncReset,
        // INITIALIZE Ctr callers
    MF625CtrRead,
    MF625CtrReset,
        // INITIALIZE PWM callers
    MF625PWMWrite,
    MF625PWMMultiphaseWrite,
    MF625PWM3Write,
        // INITIALIZE Step callers
    NULL,			// Not available
    };



/** Internal initialization function that should be called from HudaqOpenDevice
   * \todo Exclusive access is only partially implemented. */
static int Init622(DeviceRecord *DevRecord, int IniOptions)
{
MF624_Private *Cache;

  if (DevRecord==NULL) return -1;

  if (DevRecord->DrvRes.Resources.NumMemResources<2) return -2;  //insufficient amount of resources
  Cache = DevRecord->DrvRes.DriverData;
  if (Cache==NULL) return -3;
  if (DevRecord->DrvRes.DriverDataSize<sizeof(MF624_Private)) return -4;

  DevRecord->pCT = &CT622;

  if ((IniOptions & HudaqOpenNOINIT)==0)
    {       //initialize hardware only if no other application is running

      /* Initialization of AD's, DA's, DI and DO. */
    InitAiAoDiDo(DevRecord->DrvRes.Resources.MemResources[0].Base,
	         DevRecord->DrvRes.Resources.MemResources[1].Base,
		 Cache);
    }

return 1; //success
}


const CallTable CT622 =
    {
    "AD622", 0x186C, 0x0622,
    Init622,
    Done624,

    AD622SetParameter,
    AD622GetParameter,
    MF624QueryRange,

        // INITIALIZE DI callers
    MF624DIRead,
    GenericDIReadBit,           //Generic implementation
    GenericOneDIReadMultiple,
        // INITIALIZE DO callers
    MF624DOWrite,
    MF624DOWriteBit,
    MF624DOWriteMultipleBits,
    GenericDOWriteMultiple,
        // INITIALIZE AI callers
    MF624AIRead,
    MF624AIReadMultiple,
        // INITIALIZE AO callers
    MF624AOWrite,
    MF624AOWriteMultiple,
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

