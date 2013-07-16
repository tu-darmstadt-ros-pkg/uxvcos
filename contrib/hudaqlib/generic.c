/****************************************************************/
/**@file generic.c:
 * Description: Empty implementation of API layer.              *
 * Dependency: ---                                              *
 *              Copyright 2006-2007 Humusoft s.r.o.             *
 ****************************************************************/
#if defined(_WIN32) || defined(_WIN64)
  #include <windows.h>
#endif
#include <malloc.h>
#include <math.h>

#include "hudaqlib.h"
#include "hudaq_internal.h"



/////////////EMPTY/DUMMY IMPLEMENTATION STUBS///////////////////

/** Prototype for any Get Parameter function */

int DummyInit(DeviceRecord *DevRecord, int IniOptions)
{
  return -1;
}


void DummyDone(DeviceRecord *DevRecord)
{
  return;
}


//////////////////GENERIC PROCEDURES//////////////////////////////


/** Device independent implementation for all devices. */
int GenericDIReadBit(const DeviceRecord *DevRecord, unsigned channel, unsigned bit)
{
  if((DevRecord->pCT->HudaqDIRead(DevRecord,channel) & (1<<bit)) == 0)
    return 0;
  return 1;
}


/** Get data from multiple digital input channels from devices that supports only one channel. */
HUDAQSTATUS GenericOneDIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, unsigned* values)
{
unsigned InWord;
char Flag = 0;

  if (number==0 || channels==NULL || values==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return HUDAQFAILURE;

  InWord = DevRecord->pCT->HudaqDIRead(DevRecord,0);
  while(number-->0)
    {
    if (*channels==0)
      {
      *values=InWord;
      Flag |= 1;
      }
    else
      {
      *values=0;
      Flag |= 2;
      }
    values++;
    channels++;
    }

  if(Flag==3) return HUDAQPARTIAL;	//several values are valid and several are not
  if(Flag==2) return HUDAQFAILURE;	//no valid value returned
  return HUDAQSUCCESS;			//all values are valid (or no channel asked)
}


/** Device independent implementation for all devices.
  If at least one call succeeds, HUDAQPARTIAL is returned. */
HUDAQSTATUS GenericDOWriteMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned* channels, const unsigned* values)
{

HUDAQSTATUS r2, ret=HUDAQSUCCESS;
int success=0;

  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDOWrite==NULL) return HUDAQFAILURE;

  if(number==0 || channels==NULL || values==NULL) return HUDAQBADARG;

  while(number-->0)
    {
    r2 = DevRecord->pCT->HudaqDOWrite(DevRecord,*channels,*values);
    if (r2==HUDAQSUCCESS)
      success++;
    else
      ret=r2;		// store last failure in ret

    channels++; values++;
    }

  if (ret==HUDAQSUCCESS || success==0) return ret;
  return HUDAQPARTIAL;
}


/** Multiple AI reads for devices that do not support this feature. */
HUDAQSTATUS GenericAIReadMultiple(const DeviceRecord *DevRecord, unsigned number, const unsigned *channels, double *values)
{
unsigned MaxChannels;
char Flag = 0;

  if (channels==NULL || values==NULL || number==0) return HUDAQBADARG;
  if (DevRecord->pCT->HudaqAIRead==NULL) return HUDAQFAILURE; //Wrong internal table, should not occur.

  MaxChannels = (unsigned)DevRecord->pCT->HudaqGetParameter(DevRecord,0,HudaqAINUMCHANNELS);
  while(number-->0)
    {
    if(MaxChannels>*channels)
      {
      Flag |= 1;
      *values = DevRecord->pCT->HudaqAIRead(DevRecord,*channels);
      }
    else
      {
      *values = UNDEFINED_VALUE;
      Flag |= 2;
      }
    values++;
    channels++;
    }

  if(Flag==3) return HUDAQPARTIAL;	//several values are valid and several are not
  if(Flag==2) return HUDAQFAILURE;	//no valid value returned
  return HUDAQSUCCESS;			//all values are valid (or no channel asked)
}


/** Encoder reset generic caller. */
void HUDAQAPI HudaqEncReset(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return;
  if(DevRecord->pCT->HudaqEncReset==NULL) return;
  DevRecord->pCT->HudaqEncReset(DevRecord,channel);
}


/** Counter reset generic caller. */
void HUDAQAPI HudaqCtrReset(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return;
  if(DevRecord->pCT->HudaqCtrReset==NULL) return;
  DevRecord->pCT->HudaqCtrReset(DevRecord,channel);
}


//////////////////////////////////////////////////////////////////////////
/** Generic function callers for all supported devices. */

double HUDAQAPI HudaqGetParameter(HUDAQHANDLE handle, unsigned channel, HudaqParameter param)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return 0;
  if(DevRecord->pCT->HudaqGetParameter==NULL) return 0;
  return DevRecord->pCT->HudaqGetParameter(DevRecord,channel,param);
}

HUDAQSTATUS HUDAQAPI HudaqSetParameter(HUDAQHANDLE handle, unsigned channel, HudaqParameter param, double value)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqSetParameter==NULL) return HUDAQFAILURE;
  return DevRecord->pCT->HudaqSetParameter(DevRecord,channel,param,value);
}

const HudaqRange* HUDAQAPI HudaqQueryRange(HUDAQHANDLE handle, HudaqSubsystem S, unsigned item)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return NULL;
  if(DevRecord->pCT->HudaqQueryRange==NULL) return NULL;
  return DevRecord->pCT->HudaqQueryRange(DevRecord,S,item);
}

//////////////////////////////////////////////

int HUDAQAPI HudaqDIRead(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIRead==NULL) return 0;
  return DevRecord->pCT->HudaqDIRead(DevRecord,channel);
}

int HUDAQAPI HudaqDIReadBit(HUDAQHANDLE handle, unsigned channel, unsigned bit)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return 0;
  if(DevRecord->pCT->HudaqDIReadBit==NULL) return 0;
  return DevRecord->pCT->HudaqDIReadBit(DevRecord,channel,bit);
}

HUDAQSTATUS HUDAQAPI HudaqDIReadMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, unsigned* values)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDIReadMultiple==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqDIReadMultiple(DevRecord,number,channels,values);
}


HUDAQSTATUS HUDAQAPI HudaqDOWriteMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, const unsigned* values)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDOWrite==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqDOWriteMultiple(DevRecord,number,channels,values);
}

HUDAQSTATUS HUDAQAPI HudaqDOWrite(HUDAQHANDLE handle, unsigned channel, unsigned value)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqDOWrite==NULL) return HUDAQFAILURE;
  return DevRecord->pCT->HudaqDOWrite(DevRecord,channel,value);
}

void HUDAQAPI HudaqDOWriteBit(HUDAQHANDLE handle, unsigned channel, unsigned bit, int value)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return;
  if(DevRecord->pCT->HudaqDOWriteBit==NULL) return;
  DevRecord->pCT->HudaqDOWriteBit(DevRecord,channel,bit,value);
}

void HUDAQAPI HudaqDOWriteMultipleBits(HUDAQHANDLE handle, unsigned channel, unsigned mask, unsigned value)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return;
  if(DevRecord->pCT->HudaqDOWriteMultipleBits==NULL) return; // HUDAQNOTSUPPORTED;
  DevRecord->pCT->HudaqDOWriteMultipleBits(DevRecord,channel,mask,value);
}


double HUDAQAPI HudaqAIRead(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return UNDEFINED_VALUE;
  if(DevRecord->pCT->HudaqAIRead==NULL) return UNDEFINED_VALUE;
  return DevRecord->pCT->HudaqAIRead(DevRecord,channel);
}

HUDAQSTATUS HUDAQAPI HudaqAIReadMultiple(HUDAQHANDLE handle, unsigned number, const unsigned *channels, double *values)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqAIReadMultiple==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqAIReadMultiple(DevRecord,number,channels,values);
}


void HUDAQAPI HudaqAOWrite(HUDAQHANDLE handle, unsigned channel, double value)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return;
  if(DevRecord->pCT->HudaqAOWrite==NULL) return;
  DevRecord->pCT->HudaqAOWrite(DevRecord,channel,value);
}

HUDAQSTATUS HUDAQAPI HudaqAOWriteMultiple(HUDAQHANDLE handle, unsigned number, const unsigned* channels, const double* values)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqAOWriteMultiple==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqAOWriteMultiple(DevRecord,number,channels,values);
}


int HUDAQAPI HudaqEncRead(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return 0;
  if(DevRecord->pCT->HudaqEncRead==NULL) return 0;
  return DevRecord->pCT->HudaqEncRead(DevRecord,channel);
}


int HUDAQAPI HudaqCtrRead(HUDAQHANDLE handle, unsigned channel)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return 0;
  if(DevRecord->pCT->HudaqCtrRead==NULL) return 0;
  return DevRecord->pCT->HudaqCtrRead(DevRecord,channel);
}


HUDAQSTATUS HUDAQAPI HudaqPWMWrite(HUDAQHANDLE handle, unsigned channel, double frequency, double dutycycle)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqPWMWrite==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqPWMWrite(DevRecord,channel,frequency,dutycycle);
}


HUDAQSTATUS HUDAQAPI HudaqPWM3Write(HUDAQHANDLE handle, unsigned channel, double frequency, double duty1, double duty2, double duty3)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqPWM3Write==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqPWM3Write(DevRecord,channel,frequency,duty1,duty2,duty3);
}


HUDAQSTATUS HudaqPWMWriteMultiphase(HUDAQHANDLE handle, unsigned channel, unsigned phasenum, const unsigned *phases, double frequency, const double *duties)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqPWMWriteMultiphase==NULL) return HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqPWMWriteMultiphase(DevRecord,channel,phasenum,phases,frequency,duties);
}


HUDAQSTATUS HUDAQAPI HudaqStepWrite(HUDAQHANDLE handle, unsigned channel, int position)
{
  const DeviceRecord *DevRecord = HudaqGetDeviceRecord(handle);
  if(DevRecord==NULL) return HUDAQBADARG;
  if(DevRecord->pCT->HudaqStepWrite==NULL) return  HUDAQNOTSUPPORTED;
  return DevRecord->pCT->HudaqStepWrite(DevRecord,channel,position);
}


static double GenericGetParameter(const DeviceRecord *DevRecord, unsigned channel, HudaqParameter param)
{
switch(param & HudaqSubsystemMASK)
  {
//case HudaqDI:
//case HudaqDO:
//case HudaqAI:  
//case HudaqAO:  
//case HudaqEnc:
//case HudaqPWM:
//case HudaqCtr:
//case HudaqStep:
  case HudaqIRQ:
	  switch((int)param)
	  {
            case HudaqIRQ+0: 
	      return ((UserDataHeader *)DevRecord->DrvRes.DriverData)->IRQcounter;
          }
	  break;
  }
return WRONG_VALUE;
}



/** Empty call table. */
const CallTable CtDummy =
    {
    "", 0, 0,
    DummyInit,
    DummyDone,

    NULL,                       // Not available
    GenericGetParameter,
    NULL,                       // Not available

        // INITIALIZE DI callers
    NULL,
    NULL,
    NULL,
        // INITIALIZE DO callers
    NULL,
    NULL,
    NULL,
    NULL,
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



