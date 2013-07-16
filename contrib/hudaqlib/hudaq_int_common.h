/****************************************************************/
/**@file hudaq_internal.h:
 * Description: Internal hudaq header with private information. *
 * Dependency: Linux or Windows 32 and Windows 64               *
 *              Copyright 2006-2007 Humusoft s.r.o.             *
 *              Copyright 2009      J.Fojtik                    *
 ****************************************************************/

#ifndef _HUDAQ_INT_COMMON_H__
#define _HUDAQ_INT_COMMON_H__



/** Common header for all internal structures. */
typedef struct
{
  unsigned HudaqlibVersion;
  unsigned IRQcounter;
  unsigned IRQpendingFlag;
  //DWORD BoardUsers[16];       ///< this value signalises whether there is any other
                /// application that currently uses given hardware.
                ///   == 0  NO, the hardware should be reinitialised
                ///   != 0  Check GetExitCodeProcess()== STILL_ACTIVE
} UserDataHeader;


typedef int (TInitDev)
   (DeviceRecord *DevRecord, int IniOptions);
typedef void (TDoneDev)
   (DeviceRecord *DevRecord);

typedef int (THudaqXXRead)
   (const DeviceRecord *DevRecord, unsigned channel);

typedef HUDAQSTATUS (THudaqDIReadMultiple)
   (const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, unsigned* values);
typedef int (THudaqDIReadBit)
   (const DeviceRecord *DevRecord, unsigned channel, unsigned bit);

typedef HUDAQSTATUS (THudaqDOWrite)
   (const DeviceRecord *DevRecord, unsigned channel, unsigned value);
typedef void (THudaqDOWriteBit)
   (const DeviceRecord *DevRecord, unsigned channel, unsigned bit, int value);
typedef void (THudaqDOWriteMultipleBits)
   (const DeviceRecord *DevRecord, unsigned channel, unsigned mask, unsigned value);
typedef HUDAQSTATUS (THudaqDOWriteMultiple)
   (const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const unsigned* values);

typedef double (THudaqAIRead)
   (const DeviceRecord *DevRecord, unsigned channel);
typedef HUDAQSTATUS (THudaqAIReadMultiple)
   (const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, double *values);

typedef void (THudaqAOWrite)
   (const DeviceRecord *DevRecord, unsigned channel, double value);
typedef HUDAQSTATUS (THudaqAOWriteMultiple)
   (const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const double* values);

typedef double (THudaqGetParameter)
   (const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param);

typedef HUDAQSTATUS (THudaqSetParameter)
   (const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param, double value);

typedef const HudaqRange *(THudaqQueryRange)
   (const DeviceRecord *DevRecord, HudaqSubsystem S, unsigned item);

typedef  HUDAQSTATUS (THudaqStepWrite)
   (const DeviceRecord *DevRecord, unsigned channel, int position);

typedef HUDAQSTATUS (THudaqPWMWrite)
   (const DeviceRecord *DevRecord, unsigned channel, double frequency, double dutycycle);

typedef HUDAQSTATUS (THudaqReset)
   (const DeviceRecord *DevRecord, unsigned channel);

typedef HUDAQSTATUS (TPWMWriteMultiphase)
   (const DeviceRecord *DevRecord, unsigned channel, unsigned phasenum, const unsigned *phases, double frequency, const double *dutys);

typedef HUDAQSTATUS (TPWM3Write)
   (const DeviceRecord *DevRecord, unsigned channel, double frequency, double duty1f, double duty2f, double duty3f);


/** Call table of HUDAQ API
  This table should not contain NULL inside any field. Please use empty caller from generic.c
  instead. */
typedef struct CallTable
{
  const char *DeviceString;
  unsigned DeviceId, VendorId;

  TInitDev *HudaqInit;
  TDoneDev *HudaqDone;

  THudaqSetParameter *HudaqSetParameter;
  THudaqGetParameter *HudaqGetParameter;
  THudaqQueryRange *HudaqQueryRange;

  THudaqXXRead *HudaqDIRead;
  THudaqDIReadBit *HudaqDIReadBit;
  THudaqDIReadMultiple *HudaqDIReadMultiple;

  THudaqDOWrite *HudaqDOWrite;
  THudaqDOWriteBit *HudaqDOWriteBit;
  THudaqDOWriteMultipleBits *HudaqDOWriteMultipleBits;
  THudaqDOWriteMultiple *HudaqDOWriteMultiple;

  THudaqAIRead *HudaqAIRead;
  THudaqAIReadMultiple *HudaqAIReadMultiple;

  THudaqAOWrite *HudaqAOWrite;
  THudaqAOWriteMultiple *HudaqAOWriteMultiple;

  THudaqXXRead *HudaqEncRead;
  THudaqReset  *HudaqEncReset;

  THudaqXXRead *HudaqCtrRead;
  THudaqReset  *HudaqCtrReset;

  THudaqPWMWrite *HudaqPWMWrite;
  TPWMWriteMultiphase *HudaqPWMWriteMultiphase;
  TPWM3Write   *HudaqPWM3Write;

  THudaqStepWrite *HudaqStepWrite;
} CallTable;

extern const CallTable CtDummy;
extern const CallTable CT612;     ///< AD612 resources
extern const CallTable CT614;     ///< MF614 resources

extern const CallTable CT622;     ///< AD622 resources
extern const CallTable CT624;     ///< MF624 resources
extern const CallTable CT625;     ///< MF625 resources

extern const CallTable CTPCI1753; ///< PCI1753 Advantech
extern const CallTable CTPCD7004; ///< PCD7004 TEDIA
extern const CallTable CTPCT7303; ///< PCT7303 TEDIA


/** Obtain DeviceRecord from Hudaq handle. */
#define HudaqGetDeviceRecord(handle) ((DeviceRecord *)(handle))

int GenericDIReadBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit);
HUDAQSTATUS GenericAIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, double *values);
HUDAQSTATUS GenericDOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const unsigned* values);
HUDAQSTATUS GenericOneDIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, unsigned* values);


#define UNDEFINED_VALUE 0       ///< result of conversion when something goes bad
#define WRONG_VALUE     0       ///< when GetParameter cannot read, return this value


#ifdef _MSC_VER
 #pragma warning(disable:4273) /* disable the "inconsistent DLL linkage" warning */
#endif //_MSC_VER


#endif	//_HUDAQ_INT_COMMON_H__
